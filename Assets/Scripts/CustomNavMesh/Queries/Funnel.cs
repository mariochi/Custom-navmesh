using Unity.Collections;
using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>
    /// "Simple Stupid Funnel Algorithm" (string-pulling): transforma um corredor de
    /// triângulos (resultado do A*) numa polilinha enxuta de waypoints, colada nos
    /// cantos dos triângulos por onde o caminho reto passaria. É o mesmo algoritmo usado
    /// pelo Detour/Recast.
    ///
    /// IMPORTANTE sobre winding: GetSharedEdge assume que os triângulos do Unity têm
    /// winding consistente (o que é verdade pra qualquer NavMesh baked pelo Unity/Recast).
    /// A convenção left/right usada abaixo foi confirmada empiricamente contra o winding
    /// real do NavMesh do Unity — antes de acertar isso o corredor saía em zigue-zague em
    /// vez de reto em espaços largos (funil "esticando" errado a cada portal).
    /// </summary>
    public static class Funnel
    {
        public static void BuildCorridor(
            float3 start,
            float3 end,
            in NativeList<int> triPath,
            in NativeArray<int3> triangles,
            in NativeArray<float3> vertices,
            NativeList<float3> outPoints)
        {
            outPoints.Add(start);

            int k = triPath.Length;
            if (k <= 1)
            {
                outPoints.Add(end);
                return;
            }

            int portalCount = k + 1; // portal 0 = (start,start) .. portal k = (end,end)
            var portalLeft = new NativeArray<float3>(portalCount, Allocator.Temp);
            var portalRight = new NativeArray<float3>(portalCount, Allocator.Temp);

            portalLeft[0] = start;
            portalRight[0] = start;
            for (int i = 1; i < k; i++)
            {
                GetSharedEdge(triangles[triPath[i - 1]], triangles[triPath[i]], vertices, out float3 l, out float3 r);
                portalLeft[i] = l;
                portalRight[i] = r;
            }
            portalLeft[k] = end;
            portalRight[k] = end;

            float3 apex = portalLeft[0];
            float3 left = portalLeft[0];
            float3 right = portalRight[0];
            int apexIndex = 0, leftIndex = 0, rightIndex = 0;

            for (int i = 1; i <= k; i++)
            {
                float3 pl = portalLeft[i];
                float3 pr = portalRight[i];

                // lado direito do funil
                if (TriArea2D(apex, right, pr) <= 0f)
                {
                    if (ApproxEqual(apex, right) || TriArea2D(apex, left, pr) > 0f)
                    {
                        right = pr;
                        rightIndex = i;
                    }
                    else
                    {
                        outPoints.Add(left);
                        apex = left; apexIndex = leftIndex;
                        left = apex; right = apex;
                        leftIndex = rightIndex = apexIndex;
                        i = apexIndex;
                        continue;
                    }
                }

                // lado esquerdo do funil (simétrico)
                if (TriArea2D(apex, left, pl) >= 0f)
                {
                    if (ApproxEqual(apex, left) || TriArea2D(apex, right, pl) < 0f)
                    {
                        left = pl;
                        leftIndex = i;
                    }
                    else
                    {
                        outPoints.Add(right);
                        apex = right; apexIndex = rightIndex;
                        left = apex; right = apex;
                        leftIndex = rightIndex = apexIndex;
                        i = apexIndex;
                        continue;
                    }
                }
            }

            outPoints.Add(end);

            portalLeft.Dispose();
            portalRight.Dispose();
        }

        static void GetSharedEdge(in int3 triA, in int3 triB, in NativeArray<float3> vertices, out float3 left, out float3 right)
        {
            for (int e = 0; e < 3; e++)
            {
                int a = triA[e];
                int b = triA[(e + 1) % 3];
                if (Contains(triB, a) && Contains(triB, b))
                {
                    // winding do NavMesh do Unity: invertido em relação ao que eu assumi originalmente
                    // (confirmado empiricamente — sem isso o funil produz zigue-zague em vez de reta).
                    left = vertices[a];
                    right = vertices[b];
                    return;
                }
            }

            // não deveria acontecer se triA/triB são realmente vizinhos no grafo
            right = left = vertices[triA.x];
        }

        static bool Contains(in int3 tri, int v) => tri.x == v || tri.y == v || tri.z == v;

        /// <summary>Área com sinal (2x) do triângulo (a,b,c) projetado no plano XZ.</summary>
        static float TriArea2D(float3 a, float3 b, float3 c)
        {
            float ax = b.x - a.x, az = b.z - a.z;
            float bx = c.x - a.x, bz = c.z - a.z;
            return bx * az - ax * bz;
        }

        static bool ApproxEqual(float3 a, float3 b) => math.distancesq(a, b) < 1e-6f;
    }
}
