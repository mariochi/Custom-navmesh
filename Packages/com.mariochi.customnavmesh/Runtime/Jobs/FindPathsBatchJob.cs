using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>
    /// Job batelado de pathfinding: cada iteração (Execute(index)) resolve UM PathRequest
    /// inteiro — localizar triângulo de início/fim, A* sobre o grafo de triângulos (mais as
    /// arestas virtuais de NavMeshLink, se houver) e funnel/string-pulling pra gerar o
    /// corredor final de waypoints. Tudo Burst-compiled, rodando em paralelo entre os
    /// requests do frame nos worker threads.
    ///
    /// É esse job que substitui o NavMesh.CalculatePath do Unity (que é síncrono e preso
    /// à main thread): aqui N agentes pedindo caminho no mesmo frame são resolvidos em
    /// paralelo, e o resultado só é sincronizado de volta quando o NavMeshJobManager dá
    /// Complete() no JobHandle.
    ///
    /// INVARIANTE IMPORTANTE: os arrays de saída (CorridorOut/CorridorLengthOut/StatusOut/
    /// CorridorIsLinkArrivalOut) são indexados por PathRequest.AgentIndex, não pelo índice
    /// do job (`index`). Como o Unity não consegue provar estaticamente que dois requests
    /// do mesmo batch nunca têm o mesmo AgentIndex, os campos de saída usam
    /// [NativeDisableParallelForRestriction]. Quem monta o batch (NavMeshJobManager)
    /// PRECISA garantir que cada agente apareça no máximo uma vez por batch — do contrário
    /// há uma corrida de dados real.
    /// </summary>
    [BurstCompile]
    public struct FindPathsBatchJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<PathRequest> Requests;

        [ReadOnly] public NativeArray<float3> Vertices;
        [ReadOnly] public NativeArray<int3> Triangles;
        [ReadOnly] public NativeArray<int3> Neighbors;
        [ReadOnly] public NativeArray<float3> Centers;
        [ReadOnly] public NativeArray<float> AreaCost;
        [ReadOnly] public NativeArray<byte> TriangleArea;
        [ReadOnly] public NavMeshSpatialGrid Grid;

        // --- off-mesh links (NavMeshLink) — arestas extras além da adjacência normal.
        // Arrays de tamanho 0 quando não há link nenhum na cena (loop vira no-op).
        [ReadOnly] public NativeArray<float3> LinkStart;
        [ReadOnly] public NativeArray<float3> LinkEnd;
        [ReadOnly] public NativeArray<int> LinkFromTriangle;
        [ReadOnly] public NativeArray<int> LinkToTriangle;
        [ReadOnly] public NativeArray<float> LinkCost;
        [ReadOnly] public NativeArray<byte> LinkArea;
        /// <summary>NavMeshLink.width de cada link — usado só pra dar variação lateral entre agentes cruzando o mesmo link ao mesmo tempo (ver BuildCorridorWithLinks). 0 = sem largura, todo mundo mira a linha central (comportamento de antes deste campo existir).</summary>
        [ReadOnly] public NativeArray<float> LinkWidth;

        [NativeDisableParallelForRestriction] public NativeArray<float3> CorridorOut;    // flat: AgentIndex * MaxCorridorPoints + i
        [NativeDisableParallelForRestriction] public NativeArray<bool> CorridorIsLinkArrivalOut; // mesmo layout: true = chegar nesse ponto é pousar de um NavMeshLink
        [NativeDisableParallelForRestriction] public NativeArray<int> CorridorLengthOut; // por AgentIndex
        [NativeDisableParallelForRestriction] public NativeArray<byte> StatusOut;        // por AgentIndex

        public void Execute(int index)
        {
            PathRequest req = Requests[index];
            int outBase = req.AgentIndex * NavMeshJobConstants.MaxCorridorPoints;

            // posição lateral determinística deste agente dentro da largura de qualquer
            // NavMeshLink que ele cruzar (ver BuildCorridorWithLinks) — sequência da razão
            // áurea sobre AgentIndex, bem distribuída em [-1,1) sem precisar coordenar com
            // outros agentes (cada índice cai numa posição diferente e espalhada, não
            // clusterizada, conforme mais agentes usam o mesmo link).
            float linkLateralT = math.frac(req.AgentIndex * 0.6180339887f) * 2f - 1f;

            int startTri = NavMeshQueryUtil.FindNearestTriangle(req.Start, Grid, Vertices, Triangles, out float3 startPoint);
            int endTri = NavMeshQueryUtil.FindNearestTriangle(req.End, Grid, Vertices, Triangles, out float3 endPoint);

            if (startTri < 0 || endTri < 0)
            {
                StatusOut[req.AgentIndex] = (byte)PathStatus.Invalid;
                CorridorLengthOut[req.AgentIndex] = 0;
                return;
            }

            // valida a máscara de área ANTES do atalho "mesmo triângulo" — sem isso, um
            // start/end que caem no mesmo triângulo retornava Success mesmo se aquele
            // triângulo estivesse fora de AreaMask (o atalho nunca passava pelo
            // IsAreaAllowed que a expansão do A* abaixo já checa nos vizinhos).
            if (!NavMeshQueryUtil.IsAreaAllowed(TriangleArea[startTri], req.AreaMask) ||
                !NavMeshQueryUtil.IsAreaAllowed(TriangleArea[endTri], req.AreaMask))
            {
                StatusOut[req.AgentIndex] = (byte)PathStatus.Invalid;
                CorridorLengthOut[req.AgentIndex] = 0;
                return;
            }

            if (startTri == endTri)
            {
                CorridorOut[outBase + 0] = startPoint;
                CorridorOut[outBase + 1] = endPoint;
                CorridorIsLinkArrivalOut[outBase + 0] = false;
                CorridorIsLinkArrivalOut[outBase + 1] = false;
                CorridorLengthOut[req.AgentIndex] = 2;
                StatusOut[req.AgentIndex] = (byte)PathStatus.Success;
                return;
            }

            int n = Triangles.Length;
            var gScore = new NativeArray<float>(n, Allocator.Temp);
            var cameFrom = new NativeArray<int>(n, Allocator.Temp);
            // -1 = triângulo alcançado via adjacência normal (ou é o próprio início); >= 0 =
            // índice do NavMeshLink usado pra chegar nesse triângulo a partir de cameFrom[].
            var cameFromLink = new NativeArray<int>(n, Allocator.Temp);
            var closed = new NativeArray<bool>(n, Allocator.Temp);
            for (int i = 0; i < n; i++)
            {
                gScore[i] = float.MaxValue;
                cameFrom[i] = -1;
                cameFromLink[i] = -1;
            }

            var heap = new NativeMinHeap(64, Allocator.Temp);
            gScore[startTri] = 0f;
            heap.Push(startTri, math.distance(Centers[startTri], Centers[endTri]));

            bool found = false;

            // rastreia, entre todos os triângulos efetivamente fechados pelo A*, qual fica
            // heuristicamente mais perto do destino — usado como alvo de um corredor de
            // "melhor esforço" se a busca esgotar sem alcançar endTri (ilha desconectada).
            // Sem isso, um destino do outro lado de um buraco/parede sem ligação deixava o
            // agente parado (NoPath), mesmo havendo um caminho válido até bem perto dele.
            int bestTri = startTri;
            float bestHeuristic = math.distance(Centers[startTri], Centers[endTri]);

            while (heap.Count > 0)
            {
                int current = heap.Pop();
                if (closed[current]) continue; // lazy deletion de entradas duplicadas no heap
                closed[current] = true;

                if (current == endTri)
                {
                    found = true;
                    break;
                }

                float h = math.distance(Centers[current], Centers[endTri]);
                if (h < bestHeuristic)
                {
                    bestHeuristic = h;
                    bestTri = current;
                }

                int3 nbs = Neighbors[current];
                for (int e = 0; e < 3; e++)
                {
                    int nb = nbs[e];
                    if (nb < 0 || closed[nb]) continue;
                    if (!NavMeshQueryUtil.IsAreaAllowed(TriangleArea[nb], req.AreaMask)) continue;

                    float stepCost = math.distance(Centers[current], Centers[nb]) * math.max(AreaCost[TriangleArea[nb]], 0.01f);
                    float tentativeG = gScore[current] + stepCost;

                    if (tentativeG < gScore[nb])
                    {
                        cameFrom[nb] = current;
                        cameFromLink[nb] = -1;
                        gScore[nb] = tentativeG;
                        float f = tentativeG + math.distance(Centers[nb], Centers[endTri]);
                        heap.Push(nb, f);
                    }
                }

                // arestas extras de NavMeshLink que partem de 'current' — varredura linear
                // sobre todos os links da cena (tipicamente dezenas, não milhares; barato
                // comparado ao resto do A* mesmo sem índice por triângulo).
                for (int lk = 0; lk < LinkFromTriangle.Length; lk++)
                {
                    if (LinkFromTriangle[lk] != current) continue;

                    int nb = LinkToTriangle[lk];
                    if (closed[nb]) continue;
                    if (!NavMeshQueryUtil.IsAreaAllowed(LinkArea[lk], req.AreaMask)) continue;

                    float tentativeG = gScore[current] + LinkCost[lk];
                    if (tentativeG < gScore[nb])
                    {
                        cameFrom[nb] = current;
                        cameFromLink[nb] = lk;
                        gScore[nb] = tentativeG;
                        float f = tentativeG + math.distance(Centers[nb], Centers[endTri]);
                        heap.Push(nb, f);
                    }
                }
            }

            if (!found)
            {
                // best-effort: se o A* fechou algum triângulo além do próprio início (ou
                // seja, existe pelo menos uma aresta explorável na "ilha" de startTri), monta
                // um corredor até o ponto do triângulo mais perto do destino (heuristicamente)
                // encontrado nessa ilha, em vez de deixar o agente parado — mesma ideia do
                // NavMeshPathStatus.PathPartial nativo. Só cai em NoPath puro (corredor vazio)
                // se bestTri nunca saiu de startTri (nenhum vizinho alcançável de jeito nenhum).
                if (bestTri != startTri)
                {
                    int3 bt = Triangles[bestTri];
                    float3 bestPoint = NavMeshQueryUtil.ClosestPointOnTriangle(endPoint, Vertices[bt.x], Vertices[bt.y], Vertices[bt.z]);

                    ReconstructPath(cameFrom, cameFromLink, bestTri, out var bestTriPath, out var bestArrivalLink);

                    var bestWaypoints = new NativeList<float3>(16, Allocator.Temp);
                    var bestIsLinkArrival = new NativeList<bool>(16, Allocator.Temp);
                    BuildCorridorWithLinks(startPoint, bestPoint, bestTriPath, bestArrivalLink, bestWaypoints, bestIsLinkArrival, req.Radius, linkLateralT);

                    int bestCount = math.min(bestWaypoints.Length, NavMeshJobConstants.MaxCorridorPoints);
                    for (int i = 0; i < bestCount; i++)
                    {
                        CorridorOut[outBase + i] = bestWaypoints[i];
                        CorridorIsLinkArrivalOut[outBase + i] = bestIsLinkArrival[i];
                    }

                    CorridorLengthOut[req.AgentIndex] = bestCount;
                    StatusOut[req.AgentIndex] = (byte)PathStatus.BestEffort;

                    bestTriPath.Dispose();
                    bestArrivalLink.Dispose();
                    bestWaypoints.Dispose();
                    bestIsLinkArrival.Dispose();
                }
                else
                {
                    StatusOut[req.AgentIndex] = (byte)PathStatus.NoPath;
                    CorridorLengthOut[req.AgentIndex] = 0;
                }

                heap.Dispose();
                gScore.Dispose();
                cameFrom.Dispose();
                cameFromLink.Dispose();
                closed.Dispose();
                return;
            }

            ReconstructPath(cameFrom, cameFromLink, endTri, out var triPath, out var arrivalLink);

            var waypoints = new NativeList<float3>(16, Allocator.Temp);
            var isLinkArrival = new NativeList<bool>(16, Allocator.Temp);
            BuildCorridorWithLinks(startPoint, endPoint, triPath, arrivalLink, waypoints, isLinkArrival, req.Radius, linkLateralT);

            int count = math.min(waypoints.Length, NavMeshJobConstants.MaxCorridorPoints);
            for (int i = 0; i < count; i++)
            {
                CorridorOut[outBase + i] = waypoints[i];
                CorridorIsLinkArrivalOut[outBase + i] = isLinkArrival[i];
            }

            CorridorLengthOut[req.AgentIndex] = count;
            StatusOut[req.AgentIndex] = (byte)(waypoints.Length > NavMeshJobConstants.MaxCorridorPoints
                ? PathStatus.PartialCorridor
                : PathStatus.Success);

            heap.Dispose();
            gScore.Dispose();
            cameFrom.Dispose();
            cameFromLink.Dispose();
            closed.Dispose();
            triPath.Dispose();
            arrivalLink.Dispose();
            waypoints.Dispose();
            isLinkArrival.Dispose();
        }

        /// <summary>
        /// Caminha 'cameFrom' de trás pra frente a partir de 'targetTri' até -1 (o início),
        /// devolvendo a sequência de triângulos em ordem de percurso (start→target) e, em
        /// paralelo, o índice do NavMeshLink usado pra CHEGAR em cada um desses triângulos
        /// (arrivalLink[i], -1 se veio por adjacência normal — sempre -1 no índice 0, o
        /// próprio início, que não tem aresta de chegada).
        /// </summary>
        static void ReconstructPath(in NativeArray<int> cameFrom, in NativeArray<int> cameFromLink, int targetTri, out NativeList<int> triPath, out NativeList<int> arrivalLink)
        {
            triPath = new NativeList<int>(16, Allocator.Temp);
            arrivalLink = new NativeList<int>(16, Allocator.Temp);

            int cur = targetTri;
            while (cur != -1)
            {
                triPath.Add(cur);
                arrivalLink.Add(cameFromLink[cur]);
                cur = cameFrom[cur];
            }
            ReverseInPlace(triPath);
            ReverseInPlace(arrivalLink);
        }

        /// <summary>
        /// Monta o corredor final a partir da sequência de triângulos, rodando o funnel
        /// normal (Funnel.BuildCorridor) dentro de cada trecho contíguo ligado por adjacência
        /// real, e inserindo um "salto" reto (dois pontos, sem funil — LinkStart/LinkEnd do
        /// NavMeshLink usado, deslocados lateralmente por 'lateralT' — ver
        /// ApplyLinkLateralOffset) toda vez que 'arrivalLink' indica que o próximo triângulo
        /// foi alcançado via link em vez de aresta compartilhada. 'isLinkArrivalOut[i]' marca
        /// os pontos que são pouso de um salto — AvoidanceAndMoveJob usa isso pra saber quando
        /// NÃO tentar clampar a posição na superfície do NavMesh (o agente está atravessando
        /// o vão do link, não tem triângulo nenhum embaixo dele nesse trecho).
        /// </summary>
        void BuildCorridorWithLinks(
            float3 overallStart, float3 overallEnd,
            in NativeList<int> triPath, in NativeList<int> arrivalLink,
            NativeList<float3> waypointsOut, NativeList<bool> isLinkArrivalOut, float radius, float lateralT)
        {
            int k = triPath.Length;

            // caminho sem NENHUM link usado (o caso comum, de longe) — funil direto sobre o
            // triPath inteiro, sem nenhum overhead extra de detecção de fronteira.
            bool anyLink = false;
            for (int i = 1; i < k; i++)
            {
                if (arrivalLink[i] >= 0) { anyLink = true; break; }
            }

            if (!anyLink)
            {
                Funnel.BuildCorridor(overallStart, overallEnd, triPath, Triangles, Vertices, waypointsOut, radius);
                for (int i = 0; i < waypointsOut.Length; i++)
                    isLinkArrivalOut.Add(false);
                return;
            }

            int runStart = 0;
            float3 segStart = overallStart;

            for (int i = 1; i <= k; i++)
            {
                bool isBoundary = i == k || arrivalLink[i] >= 0;
                if (!isBoundary) continue;

                int runLen = i - runStart;
                float3 segEnd = (i == k) ? overallEnd
                    : ApplyLinkLateralOffset(LinkStart[arrivalLink[i]], LinkStart[arrivalLink[i]], LinkEnd[arrivalLink[i]], LinkWidth[arrivalLink[i]], lateralT);

                if (runLen == 1)
                {
                    // run de um único triângulo: reta direta, sem funil (mesmo atalho do
                    // "startTri == endTri" lá em Execute()). Só adiciona 'segStart' se for o
                    // PRIMEIRO run geral — senão ele é o mesmo ponto já adicionado como pouso
                    // do link anterior (isLinkArrival=true) na iteração passada.
                    if (runStart == 0)
                    {
                        waypointsOut.Add(segStart);
                        isLinkArrivalOut.Add(false);
                    }

                    waypointsOut.Add(segEnd);
                    isLinkArrivalOut.Add(false);
                }
                else
                {
                    var subPath = new NativeList<int>(runLen, Allocator.Temp);
                    for (int t = runStart; t < i; t++) subPath.Add(triPath[t]);

                    var subWaypoints = new NativeList<float3>(runLen + 1, Allocator.Temp);
                    Funnel.BuildCorridor(segStart, segEnd, subPath, Triangles, Vertices, subWaypoints, radius);

                    // subWaypoints[0] é sempre 'segStart' — só adiciona de novo se for o
                    // PRIMEIRO run geral (senão duplicaria o ponto de pouso do link anterior,
                    // já adicionado como isLinkArrival=true no fim da iteração passada).
                    int from = (runStart == 0) ? 0 : 1;
                    for (int w = from; w < subWaypoints.Length; w++)
                    {
                        waypointsOut.Add(subWaypoints[w]);
                        isLinkArrivalOut.Add(false);
                    }

                    subPath.Dispose();
                    subWaypoints.Dispose();
                }

                if (i < k)
                {
                    // insere o pouso do salto — o ponto de partida (LinkStart, == segEnd deste
                    // run) já foi adicionado acima como o último waypoint do run que fechou.
                    // Mesmo deslocamento lateral aplicado nos dois lados (segEnd acima e aqui)
                    // — o agente atravessa numa reta paralela à linha central do link, não
                    // necessariamente NA linha central, então não "desalinha" o salto.
                    float3 landingPoint = ApplyLinkLateralOffset(LinkEnd[arrivalLink[i]], LinkStart[arrivalLink[i]], LinkEnd[arrivalLink[i]], LinkWidth[arrivalLink[i]], lateralT);
                    waypointsOut.Add(landingPoint);
                    isLinkArrivalOut.Add(true);

                    segStart = landingPoint;
                    runStart = i;
                }
            }
        }

        /// <summary>
        /// Desloca 'point' (LinkStart ou LinkEnd do mesmo link) lateralmente — perpendicular
        /// à linha do link, no plano horizontal — por até metade de 'width', escalado por
        /// 'lateralT' (esperado em [-1,1], ver Execute()). Sem isso, todo agente cruzando o
        /// mesmo NavMeshLink ao mesmo tempo mirava exatamente a mesma linha central
        /// (NavMeshLink.width era lido em lugar nenhum), então vários agentes atravessando
        /// juntos ficavam visualmente sobrepostos numa fila de largura zero em vez de
        /// espalhados pela largura real do link (uma ponte, zip-line ou passagem larga).
        ///
        /// O ponto deslocado é VALIDADO contra o grid espacial (mesmo FindNearestTriangle
        /// usado em todo o resto do pathfinding) e reprojetado no triângulo real mais
        /// próximo — sem isso, um 'width' generoso poderia empurrar o ponto de pouso/partida
        /// pra fora de qualquer triângulo real (ex.: um link cuja ÁREA CAMINHÁVEL ao redor é
        /// mais estreita que o 'width' configurado), e o agente pousaria no vazio. Se a
        /// busca não achar NENHUM triângulo perto do ponto deslocado, cai de volta pro ponto
        /// original (sem deslocamento) — modo mais seguro que arriscar um pouso inválido.
        /// </summary>
        float3 ApplyLinkLateralOffset(float3 point, float3 linkStart, float3 linkEnd, float width, float lateralT)
        {
            if (width <= 0f) return point;

            float3 dir = linkEnd - linkStart;
            float len = math.length(dir);
            if (len < 1e-5f) return point; // link degenerado (pontos coincidentes) — nada a deslocar

            float3 lateral = math.normalizesafe(math.cross(new float3(0f, 1f, 0f), dir / len));
            float3 offsetPoint = point + lateral * (lateralT * width * 0.5f);

            int tri = NavMeshQueryUtil.FindNearestTriangle(offsetPoint, Grid, Vertices, Triangles, out float3 validated);
            return tri >= 0 ? validated : point;
        }

        static void ReverseInPlace(NativeList<int> list)
        {
            int i = 0, j = list.Length - 1;
            while (i < j)
            {
                int tmp = list[i];
                list[i] = list[j];
                list[j] = tmp;
                i++;
                j--;
            }
        }
    }
}
