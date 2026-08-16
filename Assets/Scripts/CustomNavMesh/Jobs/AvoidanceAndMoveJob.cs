using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Jobs;

namespace CustomNavMesh
{
    /// <summary>
    /// Job único que faz avoidance local (reciprocal-avoidance simplificado, estilo RVO)
    /// e movimento, rodando sobre um TransformAccessArray — ou seja, escreve direto no
    /// Transform dos agentes a partir de worker threads, sem passar pela main thread.
    ///
    /// Pipeline por agente, dentro de Execute:
    ///  1. Calcula velocidade preferida — de duas formas mutuamente exclusivas por agente:
    ///     - Modo corredor (SetDestination individual): avança o cursor de waypoints do
    ///       corredor (resultado do FindPathsBatchJob) e mira no próximo.
    ///     - Modo flow field (MoveGroupWithFlowField): consulta a direção de fluxo do
    ///       triângulo onde o agente está (ComputeFlowFieldJob calculou isso pra TODOS os
    ///       triângulos de uma vez, compartilhado por todo o grupo); perto do alvo troca
    ///       pra mira direta no ponto exato.
    ///  2. Consulta vizinhos via SpatialHash (grid de agentes) e ajusta a velocidade preferida
    ///     pra evitar colisão (half-plane simplificado + time-to-collision, não é uma LP ORCA
    ///     completa — ver observação no README).
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
        [ReadOnly] public NativeParallelMultiHashMap<int, int> SpatialHash;

        // --- corredores (resultado do FindPathsBatchJob) — modo individual (SetDestination) ---
        [ReadOnly] public NativeArray<float3> CorridorFlat;
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
        /// Multiplicador (0-1) do empurrão de sobreposição quando os dois agentes já andam na
        /// mesma direção (marchando juntos). 1 = sem amortecimento (empurrão cheio sempre,
        /// comportamento antigo); valores menores acalmam oscilação em multidão densa/gargalo
        /// sem enfraquecer a resposta a colisões de frente/cruzadas.
        /// </summary>
        public float CrowdPushDamping;

        /// <summary>
        /// Diagnóstico por agente pro frame atual (ver MovementFaultType) — 0/None é o caso
        /// normal. Escrito aqui, lido pelo NavMeshJobManager (main thread, pós-Complete) pra
        /// logar e mostrar no gizmo. Existe porque um travamento silencioso (NaN se propagando
        /// por comparações que sempre dão falso, ou um agente perdendo a referência de
        /// triângulo) não lançava exceção nem aparecia em lugar nenhum antes disso.
        /// </summary>
        public NativeArray<byte> MovementFault;

        public void Execute(int index, TransformAccess transform)
        {
            MovementFault[index] = (byte)MovementFaultType.None;
            float3 pos = Positions[index];
            int ffSlot = FlowFieldSlot[index];

            float3 prefVel = ffSlot >= 0
                ? ComputeFlowFieldPrefVel(index, ffSlot, pos)
                : ComputeCorridorPrefVel(index, pos);

            // Acumula a contribuição de CADA vizinho contra a MESMA velocidade preferida fixa
            // (não uma 'vel' sendo mutada a cada vizinho processado) e tira a média no final —
            // duas propriedades que fazem toda diferença em multidão densa:
            //  1. Ordem de processamento dos vizinhos deixa de afetar o resultado (todos avaliam
            //     o mesmo cenário de referência, em vez de reagir à correção parcial um do outro).
            //  2. A força total de avoidance fica limitada independente de quantos vizinhos estão
            //     por perto — antes, um agente cercado de dezenas de vizinhos somava dezenas de
            //     correções sem normalizar, o que sozinho já é receita de oscilação/zigue-zague.
            float3 avoidanceSum = float3.zero;
            int neighborCount = 0;

            // teste diagnóstico: agente em flow field pode pular o avoidance inteiro, pra isolar
            // se um zigue-zague residual vem daqui ou de outro lugar (pode se sobrepor com isso ligado).
            bool skipAvoidance = ffSlot >= 0 && FlowFieldIgnoresAvoidance;

            if (!skipAvoidance)
            {
                int2 selfCell = (int2)math.floor(pos.xz / NeighborCellSize);
                for (int dx = -1; dx <= 1; dx++)
                {
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        int key = HashCellKey(selfCell + new int2(dx, dy));
                        if (SpatialHash.TryGetFirstValue(key, out int other, out var it))
                        {
                            do
                            {
                                if (other == index) continue;
                                ApplyAvoidance(index, other, pos, prefVel, in Positions, in PrevVelocities, in Radii, in MaxSpeeds,
                                    NeighborQueryRadius, TimeHorizon, CrowdPushDamping, ref avoidanceSum, ref neighborCount);
                            }
                            while (SpatialHash.TryGetNextValue(out other, ref it));
                        }
                    }
                }
            }

            float3 newVel = neighborCount > 0 ? prefVel + avoidanceSum / neighborCount : prefVel;

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

            float newSpeed = math.length(newVel);
            if (newSpeed > MaxSpeeds[index])
                newVel = newVel / newSpeed * MaxSpeeds[index];

            // suaviza a transição a partir da velocidade do frame anterior em vez de saltar
            // direto pra velocidade desejada — é isso que tira o zigue-zague/solavanco de
            // mudanças bruscas de direção alvo (cruzar de triângulo, convergência de grupo).
            newVel = SmoothVelocity(PrevVelocities[index], newVel, MaxSpeeds[index] * SteeringAccelerationFactor);

            float3 newPos = pos + newVel * DeltaTime;
            newPos = ClampToNavMesh(index, pos, newPos); // rente à malha — usado como verdade pra simulação

            transform.position = newPos + new float3(0f, Heights[index], 0f); // só o visual sobe
            OutPositions[index] = newPos;
            OutVelocities[index] = newVel;
        }

        float3 ComputeCorridorPrefVel(int index, float3 pos)
        {
            int cursor = CorridorCursor[index];
            int len = CorridorLength[index];
            int outBase = index * NavMeshJobConstants.MaxCorridorPoints;

            while (cursor < len - 1 && math.distance(pos, CorridorFlat[outBase + cursor]) < WaypointReachDistances[index])
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

        /// <summary>
        /// Calcula a contribuição de UM vizinho e SOMA em avoidanceSum (não aplica direto na
        /// velocidade) — quem chama tira a média por neighborCount depois de somar todos, pra
        /// não deixar a força total crescer com o nº de vizinhos por perto.
        /// </summary>
        static void ApplyAvoidance(
            int selfIndex, int otherIndex, float3 selfPos, float3 selfPrefVel,
            in NativeArray<float3> positions, in NativeArray<float3> prevVelocities,
            in NativeArray<float> radii, in NativeArray<float> maxSpeeds,
            float neighborRadius, float timeHorizon, float crowdPushDamping,
            ref float3 avoidanceSum, ref int neighborCount)
        {
            float3 otherPos = positions[otherIndex];
            float3 relPos = new float3(otherPos.x - selfPos.x, 0f, otherPos.z - selfPos.z);
            float dist = math.length(relPos);
            if (dist < 1e-4f || dist > neighborRadius) return;

            float combinedRadius = radii[selfIndex] + radii[otherIndex];
            float3 otherVel = prevVelocities[otherIndex];
            // usa a velocidade PREFERIDA (fixa, igual pra todo vizinho avaliado neste frame),
            // não uma velocidade sendo mutada vizinho a vizinho — isso tira a dependência de
            // ordem que amplificava oscilação em multidão densa.
            float3 relVel = new float3(otherVel.x - selfPrefVel.x, 0f, otherVel.z - selfPrefVel.z);

            float penetration = combinedRadius - dist;
            if (penetration > 0f)
            {
                // já sobrepostos: empurra pra fora já (metade da responsabilidade, o outro agente
                // faz o mesmo cálculo do lado dele e empurra pro lado oposto).
                //
                // Amortece o empurrão quando os dois já estão andando na MESMA direção (marchando
                // juntos — o caso comum de um grupo grande no mesmo flow field, naturalmente mais
                // apertado que combinedRadius por boa parte do trajeto): sem isso, o flow field
                // puxa os dois de volta pra junto assim que o empurrão os separa, e esse cabo de
                // guerra por frame é o que aparece como zigue-zague na multidão. Colisões de frente
                // ou cruzadas (direções opostas/perpendiculares) continuam com o empurrão cheio.
                float alignment = math.dot(math.normalizesafe(selfPrefVel), math.normalizesafe(otherVel)); // -1..1, 0 se algum estiver ~parado
                float pushScale = math.lerp(1f, crowdPushDamping, math.saturate(alignment));

                float3 pushDir = relPos / dist;
                avoidanceSum -= pushDir * (penetration / math.max(timeHorizon, 0.01f)) * 0.5f * pushScale;
                neighborCount++;
                return;
            }

            float t = EstimateTimeToCollision(relPos, relVel, combinedRadius);
            if (t >= 0f && t < timeHorizon)
            {
                float3 futureRelPos = relPos + relVel * t;
                float3 avoidDir = math.normalizesafe(futureRelPos, relPos / dist);
                float urgency = 1f - (t / timeHorizon);
                avoidanceSum -= avoidDir * urgency * maxSpeeds[selfIndex] * 0.5f;
                neighborCount++;
            }
        }

        /// <summary>Menor t >= 0 tal que |relPos + relVel * t| == combinedRadius, ou -1 se nunca acontece.</summary>
        static float EstimateTimeToCollision(float3 relPos, float3 relVel, float combinedRadius)
        {
            float a = math.dot(relVel, relVel);
            float c = math.dot(relPos, relPos) - combinedRadius * combinedRadius;
            if (c <= 0f) return 0f;
            if (a < 1e-8f) return -1f;

            float b = 2f * math.dot(relPos, relVel);
            float disc = b * b - 4f * a * c;
            if (disc < 0f) return -1f;

            float sq = math.sqrt(disc);
            float t = (-b - sq) / (2f * a);
            return t >= 0f ? t : -1f;
        }

        /// <summary>
        /// 'safePos' é a posição confirmada válida do início do frame (antes de integrar
        /// velocidade) — usada como fallback se nada abaixo achar um triângulo, em vez de
        /// aceitar 'newPos' (que pode estar fora da malha ou corrompida).
        /// </summary>
        float3 ClampToNavMesh(int index, float3 safePos, float3 newPos)
        {
            int tri = CurrentTriangle[index];

            if (tri >= 0 && tri < NavTriangles.Length)
            {
                int found = TestTriangleAndNeighbors(tri, newPos, out float3 cp, out float d2);
                float threshold = NeighborQueryRadius * NeighborQueryRadius + 1f;
                if (found >= 0 && d2 <= threshold)
                {
                    CurrentTriangle[index] = found;
                    return cp;
                }
            }

            int nearest = NavMeshQueryUtil.FindNearestTriangle(newPos, TriGrid, NavVertices, NavTriangles, out float3 cp2);
            if (nearest >= 0)
            {
                CurrentTriangle[index] = nearest;
                return cp2;
            }

            // não achou NENHUM triângulo — nem no cache+vizinhos, nem na busca completa do
            // grid espacial. NÃO sobrescreve CurrentTriangle com -1 (preserva a última
            // referência válida, dá ao próximo frame a melhor chance de reconectar pelo
            // caminho rápido) e NÃO aceita 'newPos' — mantém o agente parado na última posição
            // confirmada válida. Sinaliza pro manager logar; se persistir por muitos frames,
            // é um agente genuinamente preso fora do NavMesh, não um solavanco de 1 frame.
            MovementFault[index] = (byte)MovementFaultType.LostNavMesh;
            return safePos;
        }

        int TestTriangleAndNeighbors(int tri, float3 p, out float3 closest, out float distSq)
        {
            int best = -1;
            float bestDistSq = float.MaxValue;
            float3 bestPoint = p;

            CheckOne(tri, p, ref best, ref bestDistSq, ref bestPoint);

            int3 nbs = NavNeighbors[tri];
            for (int e = 0; e < 3; e++)
            {
                int nb = nbs[e];
                if (nb >= 0) CheckOne(nb, p, ref best, ref bestDistSq, ref bestPoint);
            }

            closest = bestPoint;
            distSq = bestDistSq;
            return best;
        }

        void CheckOne(int tri, float3 p, ref int best, ref float bestDistSq, ref float3 bestPoint)
        {
            int3 t = NavTriangles[tri];
            float3 cp = NavMeshQueryUtil.ClosestPointOnTriangle(p, NavVertices[t.x], NavVertices[t.y], NavVertices[t.z]);
            float d2 = math.distancesq(cp, p);
            if (d2 < bestDistSq)
            {
                bestDistSq = d2;
                best = tri;
                bestPoint = cp;
            }
        }

        public static int HashCellKey(int2 cell) => cell.x * 92821 + cell.y * 68917;
    }
}
