using Unity.Collections;
using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>
    /// Funções estáticas Burst-compatíveis (sem alocação gerenciada, sem chamadas de API
    /// do Unity) usadas dentro dos Jobs de pathfinding/avoidance.
    /// </summary>
    public static class NavMeshQueryUtil
    {
        public static bool IsAreaAllowed(byte area, uint mask) => (mask & (1u << area)) != 0;

        /// <summary>
        /// Ponto mais próximo de <paramref name="p"/> sobre o triângulo (a,b,c), em 3D.
        /// Algoritmo clássico (Ericson, "Real-Time Collision Detection") via coordenadas baricêntricas.
        /// </summary>
        public static float3 ClosestPointOnTriangle(float3 p, float3 a, float3 b, float3 c)
        {
            float3 ab = b - a;
            float3 ac = c - a;
            float3 ap = p - a;

            float d1 = math.dot(ab, ap);
            float d2 = math.dot(ac, ap);
            if (d1 <= 0f && d2 <= 0f) return a;

            float3 bp = p - b;
            float d3 = math.dot(ab, bp);
            float d4 = math.dot(ac, bp);
            if (d3 >= 0f && d4 <= d3) return b;

            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float v = d1 / (d1 - d3);
                return a + v * ab;
            }

            float3 cp = p - c;
            float d5 = math.dot(ab, cp);
            float d6 = math.dot(ac, cp);
            if (d6 >= 0f && d5 <= d6) return c;

            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float w = d2 / (d2 - d6);
                return a + w * ac;
            }

            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return b + w * (c - b);
            }

            float denom = 1f / (va + vb + vc);
            float v2 = vb * denom;
            float w2 = vc * denom;
            return a + ab * v2 + ac * w2;
        }

        /// <summary>
        /// Acha o triângulo mais próximo de <paramref name="p"/> usando o grid espacial
        /// como broad-phase (busca em anéis de células crescentes, a partir da célula de p,
        /// parando um anel depois de achar o primeiro candidato — mínimo local seguro pra
        /// grids com célula >= tamanho médio de triângulo).
        /// </summary>
        public static int FindNearestTriangle(
            float3 p,
            in NavMeshSpatialGrid grid,
            in NativeArray<float3> vertices,
            in NativeArray<int3> triangles,
            out float3 closestPoint)
        {
            int2 center = grid.WorldToCell(p.xz);
            float bestDistSq = float.MaxValue;
            int bestTri = -1;
            float3 bestPoint = p;

            int maxRing = math.max(grid.CellCount.x, grid.CellCount.y) + 1;

            for (int ring = 0; ring <= maxRing; ring++)
            {
                int minX = center.x - ring, maxX = center.x + ring;
                int minY = center.y - ring, maxY = center.y + ring;

                for (int cx = minX; cx <= maxX; cx++)
                {
                    for (int cy = minY; cy <= maxY; cy++)
                    {
                        // só o contorno externo do quadrado (células internas já visitadas em anéis anteriores)
                        bool isBorder = cx == minX || cx == maxX || cy == minY || cy == maxY;
                        if (!isBorder) continue;

                        var cell = new int2(cx, cy);
                        if (!grid.IsValidCell(cell)) continue;

                        int key = NavMeshSpatialGrid.CellKey(cell);
                        if (grid.CellToTriangle.TryGetFirstValue(key, out int triIdx, out var it))
                        {
                            do
                            {
                                int3 tri = triangles[triIdx];
                                float3 cp = ClosestPointOnTriangle(p, vertices[tri.x], vertices[tri.y], vertices[tri.z]);
                                float d2 = math.distancesq(cp, p);
                                if (d2 < bestDistSq)
                                {
                                    bestDistSq = d2;
                                    bestTri = triIdx;
                                    bestPoint = cp;
                                }
                            }
                            while (grid.CellToTriangle.TryGetNextValue(out triIdx, ref it));
                        }
                    }
                }

                // já achamos algo: mais um anel de margem (pra não perder um triângulo mais
                // próximo que caia no anel seguinte) e paramos.
                if (bestTri != -1 && ring >= 1) break;
            }

            closestPoint = bestPoint;
            return bestTri;
        }

        /// <summary>
        /// Ponto médio da aresta compartilhada entre dois triângulos vizinhos. Ordem
        /// left/right não importa aqui (ao contrário de Funnel.GetSharedEdge) — só o ponto
        /// da fronteira, usado como alvo de direção do flow field em vez do centróide do
        /// vizinho (que em triângulos alongados/irregulares fica fora do eixo reto até o
        /// objetivo e produz zigue-zague ao cruzar de triângulo em triângulo).
        /// </summary>
        public static float3 GetSharedEdgeMidpoint(in int3 triA, in int3 triB, in NativeArray<float3> vertices)
        {
            float3 sum = float3.zero;
            int found = 0;

            for (int e = 0; e < 3; e++)
            {
                int v = triA[e];
                if (Contains(triB, v))
                {
                    sum += vertices[v];
                    found++;
                }
            }

            // fallback pro centróide de triA se por algum motivo não achou 2 vértices
            // compartilhados — não deveria acontecer se triA/triB são realmente vizinhos.
            return found == 2 ? sum * 0.5f : (vertices[triA.x] + vertices[triA.y] + vertices[triA.z]) / 3f;
        }

        static bool Contains(in int3 tri, int v) => tri.x == v || tri.y == v || tri.z == v;
    }
}
