using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Jobs;

namespace CustomNavMesh
{
    /// <summary>
    /// Job único que faz avoidance local (ORCA — ver item 2 abaixo) e movimento, rodando
    /// sobre um TransformAccessArray — ou seja, escreve direto no Transform dos agentes a
    /// partir de worker threads, sem passar pela main thread.
    ///
    /// Pipeline por agente, dentro de Execute:
    ///  1. Calcula velocidade preferida — de duas formas mutuamente exclusivas por agente:
    ///     - Modo corredor (SetDestination individual): avança o cursor de waypoints do
    ///       corredor (resultado do FindPathsBatchJob) e mira no próximo.
    ///     - Modo flow field (MoveGroupWithFlowField): consulta a direção de fluxo do
    ///       triângulo onde o agente está (ComputeFlowFieldJob calculou isso pra TODOS os
    ///       triângulos de uma vez, compartilhado por todo o grupo); perto do alvo troca
    ///       pra mira direta no ponto exato.
    ///  2. Consulta vizinhos via SpatialHash (grid de agentes E de CustomNavMeshObstacle —
    ///     mesmo hashmap, obstáculo codificado como índice negativo) e resolve ORCA (Optimal
    ///     Reciprocal Collision Avoidance — Van den Berg, Guy, Lin, Manocha, "Reciprocal
    ///     n-Body Collision Avoidance", 2011): cada vizinho vira uma restrição de half-plane
    ///     no espaço de velocidades 2D (XZ), e a velocidade final é o ponto dentro de TODAS
    ///     essas restrições (e do círculo de MaxSpeed) mais próximo da velocidade preferida,
    ///     resolvido por programação linear sequencial (ver LinearProgram1/2/3 abaixo). Contra
    ///     outro agente a responsabilidade pelo desvio é 50/50 (reciprocidade real); contra um
    ///     obstáculo dinâmico é 100% do agente (o obstáculo não sabe nada de ORCA).
    ///  3. Integra a posição (Euler simples) e faz um "clamp" na superfície do NavMesh, testando
    ///     primeiro o triângulo em cache (+ vizinhos) e só caindo pro grid espacial completo se
    ///     precisar — isso é o que impede o avoidance de empurrar o agente pra fora do NavMesh
    ///     e mantém o agente colado em rampas/escadas. Esse mesmo triângulo em cache
    ///     (CurrentTriangle) é reaproveitado pra consulta do flow field, sem custo extra.
    ///
    /// Lê Positions/PrevVelocities (snapshot do fim do frame anterior, [ReadOnly]) em vez do
    /// próprio array de saída, evitando corrida de dados entre iterações paralelas que, do
    /// contrário, leriam a velocidade/posição de um vizinho ora antiga ora já atualizada
    /// dependendo da ordem de execução das threads.
    /// </summary>
    [BurstCompile]
    public struct AvoidanceAndMoveJob : IJobParallelForTransform
    {
        // --- estado de agentes (snapshot do início do frame) ---
        [ReadOnly] public NativeArray<float3> Positions;
        [ReadOnly] public NativeArray<float3> PrevVelocities;
        [ReadOnly] public NativeArray<float> Radii;
        [ReadOnly] public NativeArray<float> MaxSpeeds;
        [ReadOnly] public NativeParallelMultiHashMap<long, int> SpatialHash;

        // --- obstáculos dinâmicos (CustomNavMeshObstacle) — no MESMO SpatialHash acima,
        // codificados como índice NEGATIVO (-(obstacleIndex+1), ver BuildObstacleSpatialHashJob)
        // pra não precisar de um segundo hashmap. Arrays de tamanho 0 quando não há obstáculo
        // nenhum registrado (o decode simplesmente nunca acontece, sem custo extra).
        [ReadOnly] public NativeArray<float3> ObstaclePositions;
        [ReadOnly] public NativeArray<float3> ObstacleVelocities;
        [ReadOnly] public NativeArray<float> ObstacleRadii;

        // --- corredores (resultado do FindPathsBatchJob) — modo individual (SetDestination) ---
        [ReadOnly] public NativeArray<float3> CorridorFlat;
        /// <summary>Mesmo layout de CorridorFlat: true[j] = chegar em CorridorFlat[j] é pousar depois de atravessar um NavMeshLink — ver uso em Execute()/ClampToNavMesh.</summary>
        [ReadOnly] public NativeArray<bool> CorridorIsLinkArrival;
        [ReadOnly] public NativeArray<int> CorridorLength;
        [ReadOnly] public NativeArray<float> WaypointReachDistances;

        // --- flow field — modo de grupo (MoveGroupWithFlowField), mutuamente exclusivo com o corredor ---
        /// <summary>Por agente: índice do slot de flow field em uso, ou -1 se o agente está no modo corredor.</summary>
        [ReadOnly] public NativeArray<int> FlowFieldSlot;
        /// <summary>Achatado: slot * FlowFieldTriangleCount + triângulo.</summary>
        [ReadOnly] public NativeArray<float3> FlowFieldDirections;
        [ReadOnly] public NativeArray<float> FlowFieldDistances;
        /// <summary>
        /// Por AGENTE (não por slot): ponto exato que ele mira perto do alvo. Normalmente é o
        /// destino bruto do comando de grupo, mas com "manter formação" ligado (default) cada
        /// agente tem o SEU próprio ponto (destino + offset relativo ao centróide do grupo no
        /// momento do comando) — é isso que evita todo mundo se aglomerar no mesmo pixel ao chegar.
        /// </summary>
        [ReadOnly] public NativeArray<float3> FlowFieldPersonalTargets;
        public int FlowFieldTriangleCount;
        /// <summary>Distância-até-o-alvo (ao longo do campo) abaixo da qual troca a direção de fluxo (granularidade de triângulo) por seek direto no ponto exato — sem isso o agente ficaria "orbitando" o centro do triângulo final.</summary>
        public float FlowFieldArriveDistance;
        /// <summary>
        /// Teste diagnóstico: se true, agentes em modo flow field pulam o avoidance inteiro
        /// (não consultam vizinhos, seguem só a direção do campo + suavização de velocidade).
        /// Isola se um zigue-zague residual vem do avoidance ou de outra coisa — com isso
        /// ligado os agentes podem se sobrepor entre si.
        /// </summary>
        public bool FlowFieldIgnoresAvoidance;

        /// <summary>
        /// Deslocamento vertical do Transform em relação ao ponto de simulação no NavMesh
        /// (equivalente ao Base Offset do NavMeshAgent padrão). Só afeta a posição final
        /// escrita no Transform — Positions/OutPositions continuam rente à malha, então
        /// pathfinding, avoidance e o clamp de superfície não são afetados por esse valor.
        /// </summary>
        [ReadOnly] public NativeArray<float> Heights;

        public NativeArray<int> CorridorCursor;

        // --- grafo do navmesh, pra clamp de superfície ---
        [ReadOnly] public NavMeshSpatialGrid TriGrid;
        [ReadOnly] public NativeArray<float3> NavVertices;
        [ReadOnly] public NativeArray<int3> NavTriangles;
        [ReadOnly] public NativeArray<int3> NavNeighbors;
        public NativeArray<int> CurrentTriangle;

        // --- saída ---
        [WriteOnly] public NativeArray<float3> OutPositions;
        [WriteOnly] public NativeArray<float3> OutVelocities;

        // --- parâmetros ---
        public float DeltaTime;
        public float NeighborCellSize;
        public float NeighborQueryRadius;
        public float TimeHorizon;

        /// <summary>
        /// Limite de variação de velocidade por segundo, como múltiplo de MaxSpeed (ex.: 8
        /// = sai do repouso até a velocidade máxima em ~1/8 s). Sem isso a velocidade salta
        /// direto pro valor desejado todo frame — inofensivo em trechos retos, mas visível
        /// como zigue-zague/solavanco toda vez que a direção alvo muda bruscamente (cruzar
        /// de triângulo no flow field, vários agentes convergindo pro mesmo ponto perto do
        /// destino, etc). Suaviza os dois modos (corredor e flow field) igualmente.
        /// </summary>
        public float SteeringAccelerationFactor;

        /// <summary>
        /// Diferença de altura (Y) acima da qual dois agentes não se enxergam pra avoidance,
        /// mesmo estando próximos em XZ — sem isso, um agente em cima de uma muralha/ponte e
        /// outro embaixo dela se desviam um do outro como se estivessem no mesmo plano.
        /// </summary>
        public float VerticalAvoidanceRange;

        /// <summary>
        /// Margem de histerese (metros) que um triângulo vizinho precisa vencer por, em
        /// distância, pra substituir o triângulo em cache no clamp de superfície
        /// (ClampToNavMesh/TestTriangleAndNeighbors) — ver comentário no campo equivalente
        /// do NavMeshJobManager (onde isso é exposto no inspector).
        /// </summary>
        public float TriangleStickyMargin;

        /// <summary>
        /// Nº máximo de saltos de adjacência (NavNeighbors) que ClampToNavMesh explora via
        /// TestTriangleBFS antes de recorrer à busca irrestrita no grid espacial inteiro —
        /// ver comentário em ClampToNavMesh pro porquê disso existir (evitar que o clamp
        /// "atravesse" uma parede fina aceitando o triângulo mais próximo em linha reta do
        /// lado errado). 0 desliga essa camada (comportamento antigo: só cache+1 anel, e
        /// direto pro fallback irrestrito).
        /// </summary>
        public int WallSafeBfsHops;

        /// <summary>
        /// Distância (metros) além da qual o Transform do agente é considerado "movido por
        /// fora" da posição que este Job escreveu nele no fim do frame anterior — ver
        /// comentário em Execute() pra o porquê disso existir. Precisa ser maior que o
        /// ruído de ponto flutuante da própria escrita (~1e-5), mas pequena o bastante pra
        /// pegar um passo de knockback/root motion real sem esperar vários frames.
        /// </summary>
        public float ExternalMoveTolerance;

        /// <summary>
        /// Diagnóstico por agente pro frame atual (ver MovementFaultType) — 0/None é o caso
        /// normal. Escrito aqui, lido pelo NavMeshJobManager (main thread, pós-Complete) pra
        /// logar e mostrar no gizmo. Existe porque um travamento silencioso (NaN se propagando
        /// por comparações que sempre dão falso, ou um agente perdendo a referência de
        /// triângulo) não lançava exceção nem aparecia em lugar nenhum antes disso.
        /// </summary>
        public NativeArray<byte> MovementFault;

        /// <summary>Agente pausado: não busca ativamente o corredor/flow field (prefVel=0), mas continua recebendo/aplicando avoidance — outros agentes o veem como obstáculo e ele reage se empurrado. Corredor/cursor/flow field ficam intocados.</summary>
        [ReadOnly] public NativeArray<bool> Paused;

        /// <summary>Agente ignora avoidance por completo (equivalente a "no obstacle avoidance") — segue reto pro alvo, atravessando outros agentes.</summary>
        [ReadOnly] public NativeArray<bool> IgnoreAvoidance;

        /// <summary>Override por agente de NeighborQueryRadius; -1 = usa o valor global. "Válido só neste frame" é responsabilidade de quem chama NavMeshJobManager.SetAvoidanceOverride (o manager rereseta pra -1 todo frame).</summary>
        [ReadOnly] public NativeArray<float> NeighborRadiusOverride;

        /// <summary>Override por agente de TimeHorizon; -1 = usa o valor global.</summary>
        [ReadOnly] public NativeArray<float> TimeHorizonOverride;

        /// <summary>Velocidade explícita (strafe/dodge/knockback/step) — só vale se HasVelocityOverride[i]. Tem prioridade sobre Paused: substitui o prefVel inteiro, corredor/flow field não são tocados.</summary>
        [ReadOnly] public NativeArray<float3> VelocityOverride;
        [ReadOnly] public NativeArray<bool> HasVelocityOverride;

        public void Execute(int index, TransformAccess transform)
        {
            MovementFault[index] = (byte)MovementFaultType.None;
            float3 pos = Positions[index];

            // 'Positions' é a fonte de verdade da simulação, mas o Transform em si só é
            // escrito por ESTE Job (transform.position = newPos + altura, no fim do
            // Execute) — nada no pipeline volta a ler transform.position pra dentro de
            // Positions. Sem essa checagem, qualquer código de gameplay que mova o
            // Transform diretamente (física de knockback, root motion de animação,
            // cutscene) seria silenciosamente IGNORADO no próximo frame: o agente "salta de
            // volta" pra posição simulada, sem log, sem aviso — a única forma correta
            // documentada de empurrar um agente é SetVelocityOverride. Em vez de descartar,
            // detecta a divergência e ADOTA a posição externa como novo ponto de partida
            // (reclampada no NavMesh, igual um Warp implícito), desde que ela ainda esteja
            // perto o bastante da malha pra reclampar — se estiver longe demais (teleporte
            // pro vazio, bug, ou geometria destruída embaixo do agente), ignora e deixa o
            // fallback normal de ClampToNavMesh mais abaixo lidar com isso.
            float3 expectedTransformPos = pos + new float3(0f, Heights[index], 0f);
            float3 actualTransformPos = transform.position;
            float toleranceSq = ExternalMoveTolerance * ExternalMoveTolerance;
            if (math.distancesq(actualTransformPos, expectedTransformPos) > toleranceSq)
            {
                float3 externalPos = actualTransformPos - new float3(0f, Heights[index], 0f);
                int externalTri = NavMeshQueryUtil.FindNearestTriangle(externalPos, TriGrid, NavVertices, NavTriangles, out float3 clampedExternal);
                if (externalTri >= 0)
                {
                    pos = clampedExternal;
                    CurrentTriangle[index] = externalTri;
                    MovementFault[index] = (byte)MovementFaultType.ExternalPositionAdopted;
                }
            }

            int ffSlot = FlowFieldSlot[index];

            // ordem de prioridade: override de velocidade > pausado > corredor/flow field normal.
            // Nos dois primeiros casos NÃO chama Compute*PrefVel — CorridorCursor/flow field ficam
            // exatamente como estavam, então quando o override parar (ou Resume() for chamado), o
            // agente retoma de onde parou. O avoidance roda igual logo abaixo em qualquer caso —
            // mesmo com velocidade explícita, o agente ainda é levemente desviado se for atravessar
            // outro agente, e outros ainda o veem como obstáculo.
            bool hasOverride = HasVelocityOverride[index];
            bool isPaused = Paused[index];
            float3 prefVel = hasOverride
                ? VelocityOverride[index]
                : (isPaused
                    ? float3.zero
                    : (ffSlot >= 0 ? ComputeFlowFieldPrefVel(index, ffSlot, pos) : ComputeCorridorPrefVel(index, pos)));

            // teste diagnóstico: agente em flow field pode pular o avoidance inteiro, pra isolar
            // se um zigue-zague residual vem daqui ou de outro lugar (pode se sobrepor com isso ligado).
            // IgnoreAvoidance é o equivalente "de produção" disso, por agente (ex.: chefes que
            // atravessam a própria tropa) — mesmo mecanismo, motivo diferente.
            bool skipAvoidance = (ffSlot >= 0 && FlowFieldIgnoresAvoidance) || IgnoreAvoidance[index];

            float3 newVel;
            if (skipAvoidance)
            {
                newVel = prefVel;
            }
            else
            {
                // resolve o raio/horizonte efetivos pra ESTE agente: override por agente (setado via
                // NavMeshJobManager.SetAvoidanceOverride, ex.: perto de um gargalo) ou o valor global.
                // ComputeOrcaVelocity, logo abaixo, escala isso pra cima automaticamente (nunca pra
                // baixo) com base no próprio Radii[index] e ajusta a janela de busca no grid
                // espacial de acordo — não precisa mais coordenar manualmente com NeighborCellSize
                // nem se preocupar com agentes de raio grande "sumindo" da detecção um do outro.
                float effectiveRadius = NeighborRadiusOverride[index] >= 0f ? NeighborRadiusOverride[index] : NeighborQueryRadius;
                float effectiveTimeHorizon = TimeHorizonOverride[index] >= 0f ? TimeHorizonOverride[index] : TimeHorizon;

                // ORCA opera só em XZ — o componente vertical de prefVel (subir/descer rampa)
                // atravessa direto pro resultado sem passar pelo avoidance, exatamente como no
                // esquema anterior (a colisão entre agentes nunca foi modelada no eixo Y).
                float2 orcaVel = ComputeOrcaVelocity(
                    index, pos.xz, prefVel.xz, PrevVelocities[index].xz, Radii[index], MaxSpeeds[index], DeltaTime,
                    in Positions, in PrevVelocities, in Radii, in SpatialHash, NeighborCellSize,
                    in ObstaclePositions, in ObstacleVelocities, in ObstacleRadii,
                    effectiveRadius, effectiveTimeHorizon, VerticalAvoidanceRange, pos.y);

                newVel = new float3(orcaVel.x, prefVel.y, orcaVel.y);
            }

            // rede de segurança: se algum cálculo acima (avoidance, tempo-até-colisão, campo de
            // fluxo) produziu NaN/Infinity, NÃO deixa propagar — a partir daqui NaN comparado
            // com qualquer coisa via '<' dá sempre falso, então a busca de triângulo mais
            // próximo nunca acharia nada e o agente ficaria travado pra sempre, em silêncio
            // (nada lança exceção em Burst por causa de NaN). Zera e sinaliza em vez disso.
            if (math.any(math.isnan(newVel)) || math.any(math.isinf(newVel)))
            {
                newVel = float3.zero;
                MovementFault[index] = (byte)MovementFaultType.InvalidVelocity;
            }

            // com override de velocidade, pula o clamp de MaxSpeed (knockback precisa poder
            // exceder a velocidade normal de corrida) e a suavização de aceleração (nenhum dos
            // usos — strafe/dodge/knockback/step — quer o atraso de rampa; é pra ser instantâneo,
            // igual '.velocity =' direto era no NavMeshAgent padrão). ClampToNavMesh, logo abaixo,
            // continua rodando incondicionalmente de qualquer forma — nada disso atravessa parede.
            if (!hasOverride)
            {
                float newSpeed = math.length(newVel);
                if (newSpeed > MaxSpeeds[index])
                    newVel = newVel / newSpeed * MaxSpeeds[index];

                // suaviza a transição a partir da velocidade do frame anterior em vez de saltar
                // direto pra velocidade desejada — é isso que tira o zigue-zague/solavanco de
                // mudanças bruscas de direção alvo (cruzar de triângulo, convergência de grupo).
                newVel = SmoothVelocity(PrevVelocities[index], newVel, MaxSpeeds[index] * SteeringAccelerationFactor);
            }

            // Se o waypoint que este agente está mirando agora é um pouso de NavMeshLink
            // (ver FindPathsBatchJob.BuildCorridorWithLinks), ele está no meio de um salto
            // reto por cima de um vão sem triângulo nenhum embaixo — ClampToNavMesh não deve
            // tentar encaixar essa posição na superfície (só se aplica no modo corredor:
            // override de velocidade, pausa e flow field não têm esse conceito de waypoint).
            bool duringLinkJump = false;
            if (!hasOverride && !isPaused && ffSlot < 0)
            {
                int len = CorridorLength[index];
                int cursor = CorridorCursor[index];
                if (len > 0 && cursor < len)
                    duringLinkJump = CorridorIsLinkArrival[index * NavMeshJobConstants.MaxCorridorPoints + cursor];
            }

            float3 newPos = pos + newVel * DeltaTime;
            newPos = ClampToNavMesh(index, pos, newPos, duringLinkJump); // rente à malha — usado como verdade pra simulação

            transform.position = newPos + new float3(0f, Heights[index], 0f); // só o visual sobe
            OutPositions[index] = newPos;
            OutVelocities[index] = newVel;
        }

        float3 ComputeCorridorPrefVel(int index, float3 pos)
        {
            int cursor = CorridorCursor[index];
            int len = CorridorLength[index];
            int outBase = index * NavMeshJobConstants.MaxCorridorPoints;

            // math.max: rede de segurança final contra um WaypointReachDistance 0/negativo
            // vazando até aqui por algum caminho não coberto pelos clamps de
            // CustomNavMeshAgent/NavMeshJobManager — com 0, essa comparação nunca seria
            // satisfeita por ponto flutuante e o cursor nunca avançaria (agente orbitando
            // o mesmo waypoint pra sempre).
            float reachDistance = math.max(1e-3f, WaypointReachDistances[index]);
            while (cursor < len - 1 && math.distance(pos, CorridorFlat[outBase + cursor]) < reachDistance)
                cursor++;
            CorridorCursor[index] = cursor;

            if (len <= 0) return float3.zero;

            float3 target = CorridorFlat[outBase + math.min(cursor, len - 1)];
            return SeekTarget(pos, target, MaxSpeeds[index]);
        }

        float3 ComputeFlowFieldPrefVel(int index, int slot, float3 pos)
        {
            int tri = CurrentTriangle[index]; // cache do frame anterior (clamp de superfície roda depois, no fim do Execute)
            if (tri < 0) return float3.zero;

            int offset = slot * FlowFieldTriangleCount + tri;
            float distToGoal = FlowFieldDistances[offset];

            // triângulo fora do alcance do campo (ilha desconectada do alvo dentro dele) — fica parado,
            // igual a um PathStatus.NoPath no modo individual.
            if (distToGoal >= float.MaxValue) return float3.zero;

            if (distToGoal <= FlowFieldArriveDistance)
            {
                // perto do alvo: abandona a granularidade de triângulo e mira direto no ponto exato,
                // senão o agente ficaria "orbitando" o centro do triângulo de destino pra sempre.
                return SeekTarget(pos, FlowFieldPersonalTargets[index], MaxSpeeds[index]);
            }

            return FlowFieldDirections[offset] * MaxSpeeds[index];
        }

        /// <summary>Aproxima 'current' de 'desired', limitado a 'maxAcceleration' unidades/s² neste frame (usa DeltaTime do job).</summary>
        float3 SmoothVelocity(float3 current, float3 desired, float maxAcceleration)
        {
            if (maxAcceleration <= 0f) return desired; // 0 = suavização desligada, comportamento antigo

            // se 'current' (velocidade do frame anterior) já estiver com NaN — de antes deste
            // fix existir, por exemplo — não tenta suavizar a partir de um valor corrompido
            // (NaN + qualquer coisa = NaN, ia propagar pra sempre). Pula direto pro valor
            // desejado (já sabemos que está limpo, checado antes desta chamada).
            if (math.any(math.isnan(current)))
                return desired;

            float3 delta = desired - current;
            float deltaLen = math.length(delta);
            float maxDelta = maxAcceleration * DeltaTime;
            if (deltaLen <= maxDelta || deltaLen < 1e-5f) return desired;

            return current + delta / deltaLen * maxDelta;
        }

        /// <summary>Direção até o alvo, com velocidade limitada pra não ultrapassar ele num único frame (evita oscilar em cima do ponto final).</summary>
        float3 SeekTarget(float3 pos, float3 target, float maxSpeed)
        {
            float3 toTarget = target - pos;
            float dist = math.length(toTarget);
            if (dist <= 1e-4f) return float3.zero;

            float speed = math.min(maxSpeed, dist / math.max(DeltaTime, 1e-4f));
            return (toTarget / dist) * speed;
        }

        /// <summary>Uma restrição de half-plane no espaço de velocidades 2D (XZ): velocidades no lado permitido de (Point, Direction) evitam colisão com o vizinho que originou a reta.</summary>
        struct OrcaLine
        {
            public float2 Point;
            public float2 Direction; // unitário
        }

        /// <summary>
        /// Monta uma restrição ORCA por vizinho dentro da janela 3x3 do spatial hash e resolve
        /// a velocidade final via programação linear 2D (LinearProgram2, com fallback pra
        /// LinearProgram3 se o conjunto de restrições for inviável — sempre devolve algo,
        /// nunca "falha"). Substitui a antiga soma de empurrões + média: aqui cada vizinho
        /// reciprocamente ciente (ambos os lados rodam o mesmo cálculo, cada um assumindo
        /// metade da responsabilidade — ver ComputeOrcaLine) impõe uma restrição formal de
        /// não-colisão dentro de TimeHorizon, em vez de uma força ad-hoc somada e amortecida.
        ///
        /// IMPORTANTE (determinismo): LinearProgram1/2/3 são um método de relaxação
        /// SEQUENCIAL — a ORDEM em que as retas entram na lista afeta qual ponto exato é
        /// escolhido quando o conjunto de restrições tem mais de uma solução na fronteira
        /// (comum; é o mesmo comportamento da RVO2 original, não um bug desta implementação).
        /// Se essa ordem viesse direto da iteração do spatialHash (TryGetFirstValue/
        /// TryGetNextValue), o resultado dependeria da ordem de INSERÇÃO paralela de
        /// BuildAgentSpatialHashJob/BuildObstacleSpatialHashJob (IJobParallelFor com múltiplos
        /// worker threads escrevendo no mesmo hashmap) — que não é garantida estável entre
        /// execuções idênticas (depende de qual thread chegou primeiro em cada bucket), então
        /// o resultado do ORCA (e, por extensão, a posição final do agente) poderia variar
        /// bit a bit entre replays do mesmo estado — inaceitável pra replay/rollback
        /// determinístico. Por isso os candidatos são coletados primeiro (SEM construir a
        /// reta ainda) e ORDENADOS por índice bruto antes de virar restrição — a ordem final
        /// passa a depender só do CONJUNTO de vizinhos (sempre o mesmo pro mesmo estado),
        /// nunca da ordem de chegada em memória.
        /// </summary>
        static float2 ComputeOrcaVelocity(
            int selfIndex, float2 selfPos2D, float2 selfPrefVel2D, float2 selfCurVel2D, float selfRadius, float maxSpeed, float deltaTime,
            in NativeArray<float3> positions, in NativeArray<float3> prevVelocities, in NativeArray<float> radii,
            in NativeParallelMultiHashMap<long, int> spatialHash, float cellSize,
            in NativeArray<float3> obstaclePositions, in NativeArray<float3> obstacleVelocities, in NativeArray<float> obstacleRadii,
            float neighborRadius, float timeHorizon, float verticalAvoidanceRange, float selfY)
        {
            // Piso pra evitar uma "parede invisível" de detecção: NeighborQueryRadius é um
            // valor GLOBAL (ou por-agente via SetAvoidanceOverride), mas nunca escalava com
            // o Radius do próprio agente — dois agentes com Radius bem maior que o padrão
            // (ex.: 5 cada, raio combinado 10) podiam já estar profundamente sobrepostos
            // fisicamente e MESMO ASSIM nunca virarem candidato um do outro, se a distância
            // real entre eles excedesse o NeighborQueryRadius configurado (que não sabia
            // nada sobre o tamanho deles). math.max garante um mínimo de 2×Radius (dois
            // agentes do mesmo tamanho, encostando, sempre caem dentro do raio de query um
            // do outro) sem nunca ENCOLHER um NeighborQueryRadius configurado
            // explicitamente maior — só corrige o caso patológico, não muda o custo/
            // comportamento padrão pra agentes de raio normal (0.5 × 2 = 1, bem abaixo do
            // default de 4, então max(4,1)=4, sem regressão nenhuma).
            float effectiveNeighborRadius = math.max(neighborRadius, selfRadius * 2f);

            // A janela de busca no grid espacial também precisa escalar com esse raio
            // efetivo — antes era uma janela 3x3 FIXA (±1 célula), então um raio efetivo
            // maior que ~1.5×cellSize simplesmente não enxergava candidatos além dela,
            // mesmo que o raio de distância abaixo os permitisse. ceil garante que a busca
            // sempre cobre pelo menos effectiveNeighborRadius de raio, qualquer que seja a
            // relação entre ele e NeighborCellSize.
            int ringRadius = math.max(1, (int)math.ceil(effectiveNeighborRadius / cellSize));

            var candidates = new NativeList<int>(8, Allocator.Temp);

            int2 selfCell = (int2)math.floor(selfPos2D / cellSize);
            for (int dx = -ringRadius; dx <= ringRadius; dx++)
            {
                for (int dy = -ringRadius; dy <= ringRadius; dy++)
                {
                    long key = HashCellKey(selfCell + new int2(dx, dy));
                    if (spatialHash.TryGetFirstValue(key, out int other, out var it))
                    {
                        do candidates.Add(other);
                        while (spatialHash.TryGetNextValue(out other, ref it));
                    }
                }
            }

            // ordena por índice bruto (agentes >= 0, obstáculos < 0 — codificação de
            // BuildObstacleSpatialHashJob) ANTES de montar qualquer reta: fixa uma ordem
            // canônica que só depende de QUEM são os vizinhos, nunca de em que ordem o
            // hashmap os devolveu (ver comentário do método).
            candidates.AsArray().Sort();

            var orcaLines = new NativeList<OrcaLine>(candidates.Length, Allocator.Temp);
            for (int c = 0; c < candidates.Length; c++)
            {
                int other = candidates[c];

                float3 otherPos3;
                float otherRadius;
                float2 otherVel2D;
                float responsibility;

                if (other >= 0)
                {
                    // vizinho é outro CustomNavMeshAgent — reciprocidade real, cada
                    // lado assume metade do ajuste (ver ComputeOrcaLine).
                    if (other == selfIndex) continue;

                    otherPos3 = positions[other];
                    otherRadius = radii[other];
                    otherVel2D = prevVelocities[other].xz;
                    responsibility = 0.5f;
                }
                else
                {
                    // vizinho é um CustomNavMeshObstacle (índice codificado como
                    // negativo) — não sabe nada de ORCA e não desvia de ninguém, então
                    // a responsabilidade pelo ajuste é 100% do agente (u inteiro, não
                    // u/2), igual RVO2 trata obstáculos estáticos/não-recíprocos.
                    int obstacleIndex = -other - 1;
                    otherPos3 = obstaclePositions[obstacleIndex];
                    otherRadius = obstacleRadii[obstacleIndex];
                    otherVel2D = obstacleVelocities[obstacleIndex].xz;
                    responsibility = 1f;
                }

                // separação vertical: sem isso, dois agentes (ou um agente e um
                // obstáculo) em níveis diferentes (um em cima de uma muralha/ponte,
                // outro embaixo) se enxergavam como vizinhos só porque a distância em
                // XZ é pequena, mesmo não podendo de fato colidir.
                if (math.abs(otherPos3.y - selfY) > verticalAvoidanceRange) continue;

                float2 relPos = otherPos3.xz - selfPos2D;
                float distSq = math.lengthsq(relPos);
                if (distSq < 1e-8f || distSq > effectiveNeighborRadius * effectiveNeighborRadius) continue;

                // math.max(0, ...) é a última linha de defesa: Radius negativo já é
                // clampado em CustomNavMeshAgent/CustomNavMeshObstacle e no
                // NavMeshJobManager, mas um valor negativo aqui inverteria parte da
                // geometria da reta ORCA (combinedRadius entra de forma LINEAR, não ao
                // quadrado, na derivação em ComputeOrcaLine) em vez de simplesmente ser
                // rejeitado — vale garantir isso também no ponto de consumo.
                float combinedRadius = math.max(0f, selfRadius) + math.max(0f, otherRadius);
                // relativeVelocity usa a velocidade ATUAL de cada lado (não a
                // preferida) — é o par (selfCurVel, otherCurVel) que define onde os
                // dois ficariam daqui a pouco se nada mudasse, que é o que ORCA
                // precisa pra decidir se/como desviar; a preferida só entra depois,
                // como alvo da LP. Pro obstáculo, otherVel2D vem da estimativa por
                // diferença de posição (ver NavMeshJobManager.RefreshObstacles).
                float2 relVel = selfCurVel2D - otherVel2D;

                orcaLines.Add(ComputeOrcaLine(relPos, relVel, combinedRadius, distSq, timeHorizon, deltaTime, selfCurVel2D, responsibility));
            }

            candidates.Dispose();

            int lineFail = LinearProgram2(orcaLines, maxSpeed, selfPrefVel2D, false, out float2 newVel);
            if (lineFail < orcaLines.Length)
                LinearProgram3(orcaLines, lineFail, maxSpeed, ref newVel);

            orcaLines.Dispose();
            return newVel;
        }

        /// <summary>
        /// Constrói a reta ORCA (Point, Direction) referente a UM vizinho, seguindo a
        /// derivação geométrica padrão do artigo original (van den Berg et al. 2011): projeta
        /// a velocidade relativa atual sobre a fronteira mais próxima do "cone de colisão"
        /// (círculo de raio combinedRadius daqui a TimeHorizon, ou as duas retas tangentes a
        /// ele) pra achar o menor ajuste 'u' que evita a colisão, e reparte esse ajuste pela
        /// responsabilidade indicada (Point = velocidade atual + responsibility * u): 0.5
        /// entre dois CustomNavMeshAgent (reciprocidade real — o vizinho resolve o mesmo
        /// cálculo do lado dele e assume a outra metade), 1.0 contra um CustomNavMeshObstacle
        /// (que não sabe nada de ORCA e não desvia de ninguém — a responsabilidade inteira é
        /// do agente, igual RVO2 trata obstáculos estáticos/não-recíprocos).
        /// </summary>
        static OrcaLine ComputeOrcaLine(float2 relativePosition, float2 relativeVelocity, float combinedRadius, float distSq, float timeHorizon, float deltaTime, float2 selfCurVel2D, float responsibility)
        {
            float combinedRadiusSq = combinedRadius * combinedRadius;
            float2 direction;
            float2 u;

            if (distSq > combinedRadiusSq)
            {
                // sem sobreposição ainda: horizonte de tempo normal.
                float invTimeHorizon = 1f / math.max(timeHorizon, 1e-4f);
                float2 w = relativeVelocity - invTimeHorizon * relativePosition;
                float wLengthSq = math.lengthsq(w);
                float dotProduct1 = math.dot(w, relativePosition);

                if (dotProduct1 < 0f && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq)
                {
                    // projeta no círculo de corte (a trajetória relativa aponta pro "fundo" do cone).
                    float wLength = math.sqrt(wLengthSq);
                    float2 unitW = w / wLength;
                    direction = new float2(unitW.y, -unitW.x);
                    u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                }
                else
                {
                    // projeta numa das duas pernas (tangentes) do cone — a escolha do lado (Det)
                    // segue de qual perna a velocidade relativa está mais perto.
                    float leg = math.sqrt(math.max(0f, distSq - combinedRadiusSq));
                    if (Det(relativePosition, w) > 0f)
                    {
                        direction = new float2(
                            relativePosition.x * leg - relativePosition.y * combinedRadius,
                            relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                    }
                    else
                    {
                        direction = -new float2(
                            relativePosition.x * leg + relativePosition.y * combinedRadius,
                            -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                    }

                    float dotProduct2 = math.dot(relativeVelocity, direction);
                    u = dotProduct2 * direction - relativeVelocity;
                }
            }
            else
            {
                // já sobrepostos: horizonte de UM passo de simulação (mais urgente que
                // TimeHorizon) — evita que dois agentes já colidindo demorem a se separar.
                float invTimeStep = 1f / math.max(deltaTime, 1e-4f);
                float2 w = relativeVelocity - invTimeStep * relativePosition;
                float wLength = math.length(w);
                // degenerado (mesma posição exata, w~0) — direção arbitrária, só pra não gerar NaN;
                // praticamente nunca acontece (exigiria posição E velocidade relativa idênticas).
                float2 unitW = wLength > 1e-8f ? w / wLength : new float2(0f, 1f);
                direction = new float2(unitW.y, -unitW.x);
                u = (combinedRadius * invTimeStep - wLength) * unitW;
            }

            return new OrcaLine { Point = selfCurVel2D + responsibility * u, Direction = direction };
        }

        static float Det(float2 a, float2 b) => a.x * b.y - a.y * b.x;

        /// <summary>Ponto ótimo na reta 'lines[lineNo]' que satisfaz o círculo de raio 'radius' e as retas 0..lineNo-1 já aceitas — false se essa reta é inteiramente inviável (sem interseção não-vazia).</summary>
        static bool LinearProgram1(NativeList<OrcaLine> lines, int lineNo, float radius, float2 optVelocity, bool directionOpt, ref float2 result)
        {
            float dotProduct = math.dot(lines[lineNo].Point, lines[lineNo].Direction);
            float discriminant = dotProduct * dotProduct + radius * radius - math.lengthsq(lines[lineNo].Point);
            if (discriminant < 0f) return false; // círculo de MaxSpeed não toca essa reta

            float sqrtDiscriminant = math.sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; i++)
            {
                float denominator = Det(lines[lineNo].Direction, lines[i].Direction);
                float numerator = Det(lines[i].Direction, lines[lineNo].Point - lines[i].Point);

                if (math.abs(denominator) <= 1e-5f)
                {
                    if (numerator < 0f) return false; // retas paralelas e incompatíveis
                    continue; // paralelas e compatíveis — reta i não restringe mais que já restringia
                }

                float t = numerator / denominator;
                if (denominator >= 0f) tRight = math.min(tRight, t);
                else tLeft = math.max(tLeft, t);

                if (tLeft > tRight) return false;
            }

            if (directionOpt)
            {
                result = math.dot(optVelocity, lines[lineNo].Direction) > 0f
                    ? lines[lineNo].Point + tRight * lines[lineNo].Direction
                    : lines[lineNo].Point + tLeft * lines[lineNo].Direction;
            }
            else
            {
                float t = math.dot(lines[lineNo].Direction, optVelocity - lines[lineNo].Point);
                if (t < tLeft) result = lines[lineNo].Point + tLeft * lines[lineNo].Direction;
                else if (t > tRight) result = lines[lineNo].Point + tRight * lines[lineNo].Direction;
                else result = lines[lineNo].Point + t * lines[lineNo].Direction;
            }

            return true;
        }

        /// <summary>Resolve a LP sobre todas as retas em sequência; devolve lines.Length se TODAS foram satisfeitas, ou o índice da primeira reta inviável (chamar LinearProgram3 a partir dali).</summary>
        static int LinearProgram2(NativeList<OrcaLine> lines, float radius, float2 optVelocity, bool directionOpt, out float2 result)
        {
            if (directionOpt) result = optVelocity * radius;
            else if (math.lengthsq(optVelocity) > radius * radius) result = math.normalizesafe(optVelocity) * radius;
            else result = optVelocity;

            for (int i = 0; i < lines.Length; i++)
            {
                if (Det(lines[i].Direction, lines[i].Point - result) > 0f)
                {
                    float2 tempResult = result;
                    if (!LinearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;
                        return i;
                    }
                }
            }

            return lines.Length;
        }

        /// <summary>
        /// Conjunto de restrições inviável (sem nenhuma velocidade que satisfaça todo mundo ao
        /// mesmo tempo — comum em multidão muito densa/gargalo apertado): relaxa progressivamente
        /// a partir de 'beginLine', minimizando o quanto cada reta é violada em vez de desistir.
        /// Sempre devolve alguma velocidade (nunca "falha") — é o que dá ao sistema uma garantia
        /// de que NENHUM cálculo de avoidance trava o agente, mesmo sem solução perfeita.
        /// </summary>
        static void LinearProgram3(NativeList<OrcaLine> lines, int beginLine, float radius, ref float2 result)
        {
            float distance = 0f;

            for (int i = beginLine; i < lines.Length; i++)
            {
                if (Det(lines[i].Direction, lines[i].Point - result) > distance)
                {
                    var projLines = new NativeList<OrcaLine>(lines.Length, Allocator.Temp);

                    for (int j = 0; j < i; j++)
                    {
                        float determinant = Det(lines[i].Direction, lines[j].Direction);
                        OrcaLine line;

                        if (math.abs(determinant) <= 1e-5f)
                        {
                            if (math.dot(lines[i].Direction, lines[j].Direction) > 0f) continue; // mesma direção — redundante
                            line.Point = 0.5f * (lines[i].Point + lines[j].Point);
                        }
                        else
                        {
                            line.Point = lines[i].Point + (Det(lines[j].Direction, lines[i].Point - lines[j].Point) / determinant) * lines[i].Direction;
                        }

                        line.Direction = math.normalizesafe(lines[j].Direction - lines[i].Direction);
                        projLines.Add(line);
                    }

                    float2 tempResult = result;
                    float2 perp = new float2(-lines[i].Direction.y, lines[i].Direction.x);
                    if (LinearProgram2(projLines, radius, perp, true, out result) < projLines.Length)
                    {
                        // não deveria acontecer em teoria (o resultado já está na região viável
                        // deste sub-programa por construção) — se acontecer, é erro de ponto
                        // flutuante; mantém o resultado anterior em vez de propagar algo pior.
                        result = tempResult;
                    }

                    projLines.Dispose();
                    distance = Det(lines[i].Direction, lines[i].Point - result);
                }
            }
        }

        /// <summary>
        /// 'safePos' é a posição confirmada válida do início do frame (antes de integrar
        /// velocidade) — usada como fallback se nada abaixo achar um triângulo, em vez de
        /// aceitar 'newPos' (que pode estar fora da malha ou corrompida). 'duringLinkJump'
        /// pula o clamp inteiro: o agente está atravessando um NavMeshLink (salto reto entre
        /// dois pontos sem triângulo nenhum embaixo no meio do caminho), então 'newPos' é
        /// aceita verbatim e CurrentTriangle fica congelado no triângulo de partida do salto
        /// — o clamp normal retoma sozinho assim que o corredor avançar pro próximo waypoint
        /// (o ponto de pouso, garantidamente perto de um triângulo real).
        ///
        /// Três camadas, da mais barata/restrita pra mais cara/permissiva:
        ///  1. Cache + 3 vizinhos diretos com histerese (TestTriangleAndNeighbors) — cobre
        ///     o movimento normal de cada frame, barato.
        ///  2. BFS limitado (TestTriangleBFS) — só entra em jogo se (1) falhar (empurrão de
        ///     avoidance grande, cruzamento de vários triângulos pequenos num frame só).
        ///     IMPORTANTE: por caminhar só por NavNeighbors (adjacência REAL da malha),
        ///     isso NUNCA pode atravessar uma parede/vão sem conexão — é exatamente aí que
        ///     não existe aresta de adjacência nenhuma. Cobre movimento mais rápido que o
        ///     normal sem herdar o risco da camada 3.
        ///  3. Busca irrestrita no grid espacial inteiro (FindNearestTriangle) — só entra
        ///     em jogo se (1) e (2) falharem os dois (teleporte/Warp, agente genuinamente
        ///     perdido). Aceita o triângulo mais próximo em linha reta, SEM considerar
        ///     adjacência — o único ponto do clamp que pode, em tese, "atravessar" uma
        ///     parede fina (aceitar o triângulo do outro lado por ser o mais próximo em
        ///     distância euclidiana). Aceito como último recurso: sem isso, um agente
        ///     recém-teleportado (ou fora do alcance da malha por qualquer motivo) nunca
        ///     reconectaria — o trade-off é preferível a deixá-lo preso pra sempre.
        /// </summary>
        float3 ClampToNavMesh(int index, float3 safePos, float3 newPos, bool duringLinkJump)
        {
            if (duringLinkJump) return newPos;

            int tri = CurrentTriangle[index];
            float threshold = NeighborQueryRadius * NeighborQueryRadius + 1f;

            if (tri >= 0 && tri < NavTriangles.Length)
            {
                int found = TestTriangleAndNeighbors(tri, newPos, TriangleStickyMargin, out float3 cp, out float d2);
                if (found >= 0 && d2 <= threshold)
                {
                    CurrentTriangle[index] = found;
                    return cp;
                }

                // camada 2: BFS por adjacência real, bem mais permissivo em alcance que o
                // 1-anel acima mas ainda incapaz de atravessar uma parede (não existe
                // NavNeighbors através de uma fronteira sem conexão real na malha).
                if (WallSafeBfsHops > 0)
                {
                    int foundBfs = TestTriangleBFS(tri, newPos, WallSafeBfsHops, out float3 cpBfs, out float d2Bfs);
                    if (foundBfs >= 0 && d2Bfs <= threshold)
                    {
                        CurrentTriangle[index] = foundBfs;
                        return cpBfs;
                    }
                }
            }

            // camada 3: última linha, sem garantia de respeitar adjacência — ver comentário acima.
            int nearest = NavMeshQueryUtil.FindNearestTriangle(newPos, TriGrid, NavVertices, NavTriangles, out float3 cp2);
            if (nearest >= 0)
            {
                CurrentTriangle[index] = nearest;
                return cp2;
            }

            // não achou NENHUM triângulo — nem no cache+vizinhos, nem no BFS, nem na busca
            // completa do grid espacial. NÃO sobrescreve CurrentTriangle com -1 (preserva a
            // última referência válida, dá ao próximo frame a melhor chance de reconectar
            // pelo caminho rápido) e NÃO aceita 'newPos' — mantém o agente parado na última
            // posição confirmada válida. Sinaliza pro manager logar; se persistir por muitos
            // frames, é um agente genuinamente preso fora do NavMesh, não um solavanco de 1 frame.
            MovementFault[index] = (byte)MovementFaultType.LostNavMesh;
            return safePos;
        }

        /// <summary>
        /// BFS bounded a partir de 'startTri', caminhando só por NavNeighbors (adjacência
        /// REAL de malha) até 'maxHops' saltos — devolve o triângulo visitado mais perto de
        /// 'p'. Ao contrário de FindNearestTriangle (que busca por distância euclidiana no
        /// grid espacial, ignorando se há caminho real até lá), isso NUNCA pode "atravessar"
        /// uma parede/vão fino: se dois pontos estão fisicamente próximos mas em lados
        /// opostos de uma parede, o NavMesh não tem NENHUMA aresta de adjacência conectando
        /// os triângulos dos dois lados (é exatamente isso que define uma parede na malha),
        /// então o BFS nunca alcança o lado errado. Cobre o caso de um empurrão de avoidance
        /// ou salto de posição grande o bastante pra sair do alcance de
        /// TestTriangleAndNeighbors (1 anel) mas ainda dentro da mesma região conectada.
        /// Sem deduplicação de visitados: o custo de revisitar um triângulo já visto é só
        /// um ClosestPointOnTriangle a mais, nunca um loop infinito (a expansão para no
        /// maxHops de qualquer forma) — mas como cada triângulo tem até 3 vizinhos, sem
        /// dedupe o tamanho da fronteira pode crescer geometricamente (até 3^hop no pior
        /// caso, quando vários triângulos revisitados reabrem os mesmos vizinhos a cada
        /// anel). MaxTotalVisits limita o custo TOTAL da busca independente de maxHops,
        /// como uma segunda trava de segurança além do limite de saltos.
        /// </summary>
        const int MaxTotalVisits = 512;

        int TestTriangleBFS(int startTri, float3 p, int maxHops, out float3 closest, out float distSq)
        {
            var frontier = new NativeList<int>(16, Allocator.Temp);
            var nextFrontier = new NativeList<int>(16, Allocator.Temp);

            int best = -1;
            float bestDistSq = float.MaxValue;
            float3 bestPoint = p;
            int totalVisits = 0;

            frontier.Add(startTri);

            for (int hop = 0; hop <= maxHops && frontier.Length > 0 && totalVisits < MaxTotalVisits; hop++)
            {
                nextFrontier.Clear();

                for (int f = 0; f < frontier.Length && totalVisits < MaxTotalVisits; f++)
                {
                    int t = frontier[f];
                    totalVisits++;

                    int3 tri = NavTriangles[t];
                    float3 cp = NavMeshQueryUtil.ClosestPointOnTriangle(p, NavVertices[tri.x], NavVertices[tri.y], NavVertices[tri.z]);
                    float d2 = math.distancesq(cp, p);
                    if (d2 < bestDistSq)
                    {
                        bestDistSq = d2;
                        best = t;
                        bestPoint = cp;
                    }

                    if (hop == maxHops) continue; // não expande mais além do limite de saltos

                    int3 nbs = NavNeighbors[t];
                    for (int e = 0; e < 3; e++)
                    {
                        int nb = nbs[e];
                        if (nb >= 0) nextFrontier.Add(nb);
                    }
                }

                var tmp = frontier;
                frontier = nextFrontier;
                nextFrontier = tmp;
            }

            closest = bestPoint;
            distSq = bestDistSq;

            frontier.Dispose();
            nextFrontier.Dispose();
            return best;
        }

        /// <summary>
        /// Testa o triângulo em cache + seus 3 vizinhos diretos e devolve o mais próximo de
        /// 'p' — COM histerese: um vizinho só derruba o triângulo atual se vencer por mais de
        /// 'stickyMargin' de distância (não apenas por estar marginalmente mais perto). Sem
        /// essa margem, em trechos com triângulos pequenos e muito próximos entre si
        /// (principalmente degraus de escada) o "vencedor" muda a cada frame só por ruído de
        /// sub-milímetro na posição — cada troca reprojeta a posição clampada (sobretudo o Y)
        /// discretamente, e como a direção do frame seguinte é calculada a partir dela, isso
        /// aparece como zigue-zague bem localizado nas bordas dos degraus.
        /// </summary>
        int TestTriangleAndNeighbors(int tri, float3 p, float stickyMargin, out float3 closest, out float distSq)
        {
            int3 t0 = NavTriangles[tri];
            float3 bestPoint = NavMeshQueryUtil.ClosestPointOnTriangle(p, NavVertices[t0.x], NavVertices[t0.y], NavVertices[t0.z]);
            float bestDist = math.distance(bestPoint, p);
            int best = tri;

            int3 nbs = NavNeighbors[tri];
            for (int e = 0; e < 3; e++)
            {
                int nb = nbs[e];
                if (nb < 0) continue;

                int3 tn = NavTriangles[nb];
                float3 cp = NavMeshQueryUtil.ClosestPointOnTriangle(p, NavVertices[tn.x], NavVertices[tn.y], NavVertices[tn.z]);
                float d = math.distance(cp, p);
                if (d + stickyMargin < bestDist)
                {
                    bestDist = d;
                    best = nb;
                    bestPoint = cp;
                }
            }

            closest = bestPoint;
            distSq = bestDist * bestDist;
            return best;
        }

        /// <summary>
        /// Chave EXATA (empacota cell.x nos 32 bits altos e cell.y nos 32 bits baixos de um
        /// long) — mesma lógica de NavMeshSpatialGrid.CellKey. Antes disso era um hash int
        /// de 32 bits (`cell.x * 92821 + cell.y * 68917`) que estourava silenciosamente
        /// (Burst não lança exceção por overflow) em mundos abertos grandes, causando
        /// colisões de célula espúrias no spatial hash de agentes/obstáculos. Ver comentário
        /// completo em NavMeshSpatialGrid.CellKey.
        /// </summary>
        public static long HashCellKey(int2 cell) => ((long)cell.x << 32) | (uint)cell.y;
    }
}
