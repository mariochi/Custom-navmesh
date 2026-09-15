using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>
    /// Calcula um flow field: parte do triângulo de destino e expande por Dijkstra até
    /// todos os triângulos alcançáveis do grafo (não para no primeiro que achar, ao
    /// contrário do A* de FindPathsBatchJob — aqui o objetivo é ter a distância-até-o-alvo
    /// de TODO triângulo, não só de um).
    ///
    /// A direção de fluxo de cada triângulo NÃO é mais um vetor fixo escolhido entre "os 3
    /// vizinhos" (isso ainda causava zigue-zague real — não só visual — porque a direção
    /// podia mudar bruscamente ao cruzar de triângulo, principalmente perto do alvo onde
    /// vários caminhos convergem). Em vez disso: propaga a distância de cada triângulo pros
    /// seus 3 vértices (mínimo entre os triângulos incidentes) e reconstrói, por triângulo,
    /// o GRADIENTE do interpolante linear que passa pelos 3 valores de vértice — como
    /// triângulos vizinhos compartilham 2 dos 3 vértices, a direção resultante concorda
    /// muito mais entre eles (mesma ideia de normal suavizada por vértice vs. normal por
    /// face). A direção de fluxo é o negativo desse gradiente (desce na distância = anda
    /// rumo ao alvo).
    ///
    /// O resultado é compartilhável por qualquer nº de agentes indo pro mesmo destino —
    /// é isso que faz valer a pena rodar uma vez só em vez de um A* por agente quando um
    /// grupo grande converge pro mesmo ponto.
    ///
    /// Escreve num slice de um buffer maior (SlotOffset = slot * nº de triângulos), porque
    /// Jobs/Burst não aceitam NativeArray de NativeArray — o pool de flow fields inteiro
    /// vive em dois NativeArrays "achatados" (ver NavMeshJobManager).
    ///
    /// Custo: Dijkstra sobre o grafo INTEIRO alcançável a partir do alvo, não só até o
    /// agente mais próximo — ou seja, o custo não cai mesmo se só 1 agente usar o campo.
    /// Só compensa sobre FindPathsBatchJob quando VÁRIOS agentes convergem pro mesmo ponto.
    ///
    /// NavMeshLink: o Dijkstra também atravessa as arestas extras de NavMeshLink (mesmas
    /// que FindPathsBatchJob usa no A* individual — ver NavMeshGraph.LinkStart/...), no
    /// sentido inverso da travessia real (aqui expandimos do alvo pra fora). Sem isso,
    /// qualquer grupo cujo caminho mais curto precisasse atravessar um link (portão
    /// levadiço, ponte, rampa) nunca alcançava esse lado do grafo pelo flow field —
    /// DistanceOut ficava float.MaxValue pra sempre mesmo com o pathfinding individual
    /// achando caminho normalmente pro mesmo par de pontos (o A* individual usa links; o
    /// Dijkstra antes desta correção não usava). O triângulo de PARTIDA de cada link usado
    /// tem sua direção de fluxo apontando direto pro LinkStart (ver LinkOut), não pro
    /// gradiente de vértice normal — não existe aresta/vértice compartilhado do outro lado
    /// de um link pra interpolar. Mesmo assim, o flow field NÃO sabe atravessar o vão do
    /// link sozinho (isso é exclusivo do pipeline individual/corredor — ver
    /// AvoidanceAndMoveJob e NavMeshJobManager.CheckFlowFieldArrivals); LinkOut é o sinal
    /// que o chamador usa pra promover o agente pro pipeline individual assim que ele
    /// chega nesse triângulo de partida, não só perto do alvo final.
    /// </summary>
    [BurstCompile]
    public struct ComputeFlowFieldJob : IJob
    {
        [ReadOnly] public NativeArray<float3> Centers;
        [ReadOnly] public NativeArray<int3> Neighbors;
        [ReadOnly] public NativeArray<float> AreaCost;
        [ReadOnly] public NativeArray<byte> TriangleArea;
        [ReadOnly] public NativeArray<int3> Triangles;
        [ReadOnly] public NativeArray<float3> Vertices;

        // --- off-mesh links (NavMeshLink) — mesmas arestas extras que FindPathsBatchJob usa
        // no A* individual (ver NavMeshGraph.LinkStart/... e comentário grande abaixo, no
        // laço de Dijkstra). Arrays de tamanho 0 quando a cena não tem nenhum link.
        [ReadOnly] public NativeArray<float3> LinkStart;
        [ReadOnly] public NativeArray<int> LinkFromTriangle;
        [ReadOnly] public NativeArray<int> LinkToTriangle;
        [ReadOnly] public NativeArray<float> LinkCost;
        [ReadOnly] public NativeArray<byte> LinkArea;

        public int TargetTriangle;
        public int SlotOffset;
        public uint AreaMask;

        /// <summary>
        /// Nº de passadas de suavização (blur) aplicadas sobre a direção final, cada uma
        /// misturando a direção de cada triângulo com a dos vizinhos válidos (mesma ideia de
        /// suavização de normais por vértice). 0 desliga (comportamento antigo, só o gradiente
        /// cru). Ver comentário grande acima de SmoothDirections() pro motivo disso existir:
        /// o gradiente por triângulo já é bem mais suave que o método antigo (vetor fixo pro
        /// melhor vizinho), mas ainda herda um viés de grade de baixa frequência de COMO o
        /// Dijkstra andou pela malha quando a triangulação é regular/alinhada a grid (chão em
        /// tiles) — o blur ataca justamente essa componente de baixa frequência, sem precisar
        /// refazer o Dijkstra nem pagar um funil por agente.
        /// </summary>
        public int DirectionSmoothingIterations;

        public NativeArray<float3> DirectionsOut; // tamanho maxFlowFields * nº de triângulos
        public NativeArray<float> DistanceOut;
        /// <summary>
        /// Flat (mesmo layout de DistanceOut): índice do NavMeshLink que a rota ótima DESTE
        /// triângulo usa pra continuar em direção ao alvo, ou -1 se não precisa de nenhum. O
        /// flow field não sabe atravessar o vão de um link sozinho (não tem noção de "salto" —
        /// isso é exclusivo do pipeline individual/corredor, ver AvoidanceAndMoveJob); é essa
        /// saída que permite ao chamador (NavMeshJobManager.CheckFlowFieldArrivals) promover o
        /// agente pro pipeline individual assim que ele chega no triângulo de partida de um
        /// link — não só perto do alvo final, como já acontecia antes desta saída existir.
        /// </summary>
        public NativeArray<int> LinkOut;

        public void Execute()
        {
            int n = Centers.Length;
            for (int i = 0; i < n; i++)
            {
                DistanceOut[SlotOffset + i] = float.MaxValue;
                DirectionsOut[SlotOffset + i] = float3.zero;
                LinkOut[SlotOffset + i] = -1;
            }

            if (TargetTriangle < 0 || TargetTriangle >= n) return;

            // ---- Dijkstra sobre os triângulos (distância-até-o-alvo por triângulo) ----
            var closed = new NativeArray<bool>(n, Allocator.Temp);
            var heap = new NativeMinHeap(64, Allocator.Temp);

            // LinkOut[SlotOffset+t] = índice do NavMeshLink usado pra alcançar 'current' → 't'
            // na relaxação que deu a MELHOR distância conhecida de 't' até agora, ou -1 se a
            // melhor rota veio por adjacência normal de malha. Só importa pro triângulo que
            // fica do lado de PARTIDA do link (t == LinkFromTriangle[LinkOut[SlotOffset+t]]): a
            // direção de fluxo dele não pode vir do gradiente de vértice normal (não existe
            // aresta/vértice compartilhado do outro lado de um link — é um salto no vazio),
            // então DirectionsOut[t] aponta direto pro LinkStart do link (ver mais abaixo,
            // depois do laço de gradiente) — mesma ideia de "portal" que FallbackPortalDirection
            // já usa pra vizinhos de malha, só que com o ponto exato do link em vez do meio
            // da aresta compartilhada (não existe aresta aqui). É saída persistente (não só
            // local) pra NavMeshJobManager.CheckFlowFieldArrivals poder promover o agente pro
            // pipeline individual assim que ele chega nesse triângulo de partida — o flow field
            // não sabe atravessar o vão do link sozinho.

            DistanceOut[SlotOffset + TargetTriangle] = 0f;
            heap.Push(TargetTriangle, 0f);

            while (heap.Count > 0)
            {
                int current = heap.Pop();
                if (closed[current]) continue;
                closed[current] = true;

                int3 nbs = Neighbors[current];
                for (int e = 0; e < 3; e++)
                {
                    int nb = nbs[e];
                    if (nb < 0 || closed[nb]) continue;
                    if (!NavMeshQueryUtil.IsAreaAllowed(TriangleArea[nb], AreaMask)) continue;

                    // custo real de um agente se movendo nb -> current (sentido oposto ao
                    // desta expansão, que parte do alvo pra fora): usa o custo de área de
                    // 'current' porque é nele que o agente estaria entrando nesse sentido.
                    float stepCost = math.distance(Centers[current], Centers[nb]) * math.max(AreaCost[TriangleArea[current]], 0.01f);
                    float tentative = DistanceOut[SlotOffset + current] + stepCost;

                    if (tentative < DistanceOut[SlotOffset + nb])
                    {
                        DistanceOut[SlotOffset + nb] = tentative;
                        LinkOut[SlotOffset + nb] = -1; // chegou por adjacência normal — sobrescreve um link mais caro, se houver
                        heap.Push(nb, tentative);
                    }
                }

                // arestas extras de NavMeshLink que TERMINAM em 'current' (LinkToTriangle[lk]
                // == current): num agente andando de verdade (start → alvo), essa é a mesma
                // travessia que FindPathsBatchJob já considera (LinkFromTriangle[lk] →
                // LinkToTriangle[lk]) — aqui, expandindo do alvo pra fora, olhamos pro sentido
                // oposto, então quem relaxa é o lado de PARTIDA (nb = LinkFromTriangle[lk]).
                // Sem isso, qualquer grupo cujo caminho mais curto precise atravessar um link
                // (portão levadiço, ponte, rampa modelada como NavMeshLink) nunca alcança esse
                // lado do grafo pelo flow field — DistanceOut fica float.MaxValue pra sempre
                // mesmo com o pathfinding individual (que usa links) achando caminho normalmente
                // pro mesmo par de pontos. Varredura linear sobre todos os links da cena, mesmo
                // custo/escala que FindPathsBatchJob já paga (tipicamente dezenas, não milhares).
                for (int lk = 0; lk < LinkToTriangle.Length; lk++)
                {
                    if (LinkToTriangle[lk] != current) continue;

                    int nb = LinkFromTriangle[lk];
                    if (nb < 0 || closed[nb]) continue;
                    if (!NavMeshQueryUtil.IsAreaAllowed(LinkArea[lk], AreaMask)) continue;

                    float tentative = DistanceOut[SlotOffset + current] + LinkCost[lk];
                    if (tentative < DistanceOut[SlotOffset + nb])
                    {
                        DistanceOut[SlotOffset + nb] = tentative;
                        LinkOut[SlotOffset + nb] = lk;
                        heap.Push(nb, tentative);
                    }
                }
            }

            // ---- distância por vértice: mínimo entre os triângulos incidentes ----
            // (aproximação da distância geodésica real no vértice; suficiente pra reconstruir
            // um gradiente muito mais consistente entre triângulos vizinhos do que escolher
            // "o melhor vizinho" discretamente, já que 2 dos 3 vértices são compartilhados.)
            int vCount = Vertices.Length;
            var vertexDistance = new NativeArray<float>(vCount, Allocator.Temp);
            for (int i = 0; i < vCount; i++) vertexDistance[i] = float.MaxValue;

            for (int t = 0; t < n; t++)
            {
                float d = DistanceOut[SlotOffset + t];
                if (d >= float.MaxValue) continue;

                int3 tri = Triangles[t];
                if (d < vertexDistance[tri.x]) vertexDistance[tri.x] = d;
                if (d < vertexDistance[tri.y]) vertexDistance[tri.y] = d;
                if (d < vertexDistance[tri.z]) vertexDistance[tri.z] = d;
            }

            // ---- direção por triângulo: -gradiente do interpolante linear (d0,d1,d2) ----
            for (int t = 0; t < n; t++)
            {
                float dCenter = DistanceOut[SlotOffset + t];
                if (t == TargetTriangle || dCenter >= float.MaxValue) continue;

                // triângulo cuja rota ótima atravessa um NavMeshLink por aqui — não tem aresta/
                // vértice compartilhado com o outro lado do link pra interpolar gradiente
                // nenhum, então aponta direto pro ponto de partida do link (mesma ideia do
                // "salto reto" que FindPathsBatchJob já usa no corredor individual).
                int linkIdx = LinkOut[SlotOffset + t];
                if (linkIdx >= 0)
                {
                    DirectionsOut[SlotOffset + t] = math.normalizesafe(LinkStart[linkIdx] - Centers[t]);
                    continue;
                }

                int3 tri = Triangles[t];
                float d0 = vertexDistance[tri.x];
                float d1 = vertexDistance[tri.y];
                float d2 = vertexDistance[tri.z];

                if (d0 >= float.MaxValue || d1 >= float.MaxValue || d2 >= float.MaxValue)
                {
                    // vértice sem dado completo (raro, borda do alcance do campo) — cai pro
                    // método antigo (aponta pro portal do melhor vizinho) só nesse caso.
                    DirectionsOut[SlotOffset + t] = FallbackPortalDirection(t, dCenter);
                    continue;
                }

                float3 v0 = Vertices[tri.x];
                float3 v1 = Vertices[tri.y];
                float3 v2 = Vertices[tri.z];

                float3 e1 = v1 - v0;
                float3 e2 = v2 - v0;
                float delta1 = d1 - d0;
                float delta2 = d2 - d0;

                // resolve o gradiente 'grad' (vetor no plano do triângulo) tal que
                // dot(grad,e1)=delta1 e dot(grad,e2)=delta2 — sistema 2x2 via Cramer,
                // sem depender de cross product/normal (não é sensível a winding).
                float e11 = math.dot(e1, e1);
                float e22 = math.dot(e2, e2);
                float e12 = math.dot(e1, e2);
                float det = e11 * e22 - e12 * e12;

                if (math.abs(det) < 1e-8f)
                {
                    // triângulo degenerado/quase colinear — sem área suficiente pra um gradiente confiável
                    DirectionsOut[SlotOffset + t] = FallbackPortalDirection(t, dCenter);
                    continue;
                }

                float alpha = (delta1 * e22 - delta2 * e12) / det;
                float beta = (e11 * delta2 - e12 * delta1) / det;
                float3 grad = alpha * e1 + beta * e2; // gradiente da DISTÂNCIA (cresce ao se afastar do alvo)

                DirectionsOut[SlotOffset + t] = math.normalizesafe(-grad, FallbackPortalDirection(t, dCenter));
            }

            vertexDistance.Dispose();
            heap.Dispose();
            closed.Dispose();

            if (DirectionSmoothingIterations > 0) SmoothDirections(n);
        }

        /// <summary>
        /// Suaviza DirectionsOut[SlotOffset..] com N passadas de blur por vizinhança (cada
        /// triângulo vira a média — normalizada — da sua própria direção com a dos vizinhos
        /// válidos do MESMO lado do campo, ou seja, só entre triângulos com distância finita;
        /// nunca atravessa pra um vizinho fora de alcance/área proibida, então não puxa
        /// direção de um lado sem conexão real com o alvo).
        ///
        /// Por quê: o gradiente reconstruído em Execute() já resolve o zig-zag "duro" de
        /// escolher entre os 3 vizinhos discretamente (ver comentário da classe), mas a
        /// DISTÂNCIA de onde ele vem ainda é a soma de passos do Dijkstra sobre a adjacência
        /// da malha — em triangulações bem irregulares (Recast normal, chão aberto) isso mal
        /// se nota, mas quanto mais REGULAR/alinhada a grid for a triangulação (chão em tiles,
        /// voxel/tile size pequeno no bake), mais a distância favorece as direções dos EIXOS
        /// da grade, e o gradiente herda esse viés — na prática, grupos grandes convergindo
        /// pro mesmo alvo colam nas mesmas duas ou três direções "de grade" e acabam alinhados
        /// em fileiras retas/diagonais (visualmente um zig-zag "em degrau"), em vez de convergir
        /// num leque suave. Um blur local acha o meio-termo entre triângulos vizinhos
        /// (frequência alta = ruído/viés de grade) sem apagar a tendência geral rumo ao alvo
        /// (frequência baixa = real, sobrevive à média porque é compartilhada por toda a
        /// vizinhança). Custo é O(triângulos × iterações), pago uma vez por
        /// MoveGroupWithFlowField (não é hot path por frame).
        /// </summary>
        void SmoothDirections(int n)
        {
            var buffer = new NativeArray<float3>(n, Allocator.Temp);

            for (int iter = 0; iter < DirectionSmoothingIterations; iter++)
            {
                for (int t = 0; t < n; t++)
                {
                    float dCenter = DistanceOut[SlotOffset + t];
                    float3 own = DirectionsOut[SlotOffset + t];
                    if (t == TargetTriangle || dCenter >= float.MaxValue || math.all(own == float3.zero) || LinkOut[SlotOffset + t] >= 0)
                    {
                        // fora de alcance, é o próprio alvo, ou aponta pro ponto exato de partida
                        // de um NavMeshLink — nesse último caso a direção já é um "portal" preciso
                        // (ver Execute()); misturar com vizinhos de MALHA (que não sabem nada sobre
                        // o link) só desviaria o agente de entrar de fato no ponto de salto.
                        buffer[t] = own;
                        continue;
                    }

                    float3 sum = own;
                    int weight = 1;
                    int3 nbs = Neighbors[t];
                    for (int e = 0; e < 3; e++)
                    {
                        int nb = nbs[e];
                        if (nb < 0 || DistanceOut[SlotOffset + nb] >= float.MaxValue) continue;

                        float3 nbDir = DirectionsOut[SlotOffset + nb];
                        if (math.all(nbDir == float3.zero)) continue; // vizinho é o próprio alvo (direção nula por definição) — não puxa a média pra zero

                        sum += nbDir;
                        weight++;
                    }

                    buffer[t] = math.normalizesafe(sum, own);
                }

                // troca DirectionsOut <-> buffer pro conteúdo da próxima iteração ler o resultado desta
                for (int t = 0; t < n; t++) DirectionsOut[SlotOffset + t] = buffer[t];
            }

            buffer.Dispose();
        }

        /// <summary>Rede de segurança: aponta pro ponto médio da aresta compartilhada com o vizinho de menor distância (método antigo).</summary>
        float3 FallbackPortalDirection(int t, float dCenter)
        {
            int3 nbs = Neighbors[t];
            int best = -1;
            float bestDist = dCenter;

            for (int e = 0; e < 3; e++)
            {
                int nb = nbs[e];
                if (nb < 0) continue;
                float d = DistanceOut[SlotOffset + nb];
                if (d < bestDist)
                {
                    bestDist = d;
                    best = nb;
                }
            }

            if (best < 0) return float3.zero;

            float3 portalMid = NavMeshQueryUtil.GetSharedEdgeMidpoint(Triangles[t], Triangles[best], Vertices);
            return math.normalizesafe(portalMid - Centers[t]);
        }
    }
}
