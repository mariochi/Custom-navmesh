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

        public int TargetTriangle;
        public int SlotOffset;
        public uint AreaMask;

        public NativeArray<float3> DirectionsOut; // tamanho maxFlowFields * nº de triângulos
        public NativeArray<float> DistanceOut;

        public void Execute()
        {
            int n = Centers.Length;
            for (int i = 0; i < n; i++)
            {
                DistanceOut[SlotOffset + i] = float.MaxValue;
                DirectionsOut[SlotOffset + i] = float3.zero;
            }

            if (TargetTriangle < 0 || TargetTriangle >= n) return;

            // ---- Dijkstra sobre os triângulos (distância-até-o-alvo por triângulo) ----
            var closed = new NativeArray<bool>(n, Allocator.Temp);
            var heap = new NativeMinHeap(64, Allocator.Temp);

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
