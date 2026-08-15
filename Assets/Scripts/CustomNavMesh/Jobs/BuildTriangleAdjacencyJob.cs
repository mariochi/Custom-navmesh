using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>
    /// Descobre, pra cada triângulo, quais são os vizinhos que compartilham cada uma das
    /// 3 arestas. Roda uma única vez por bake (não é hot path), mas já sai Burst-compiled.
    ///
    /// Truque: uma aresta (a,b) de um triângulo é compartilhada por no máximo um outro
    /// triângulo, que vai enxergá-la como (b,a) (winding oposto). Usamos uma chave
    /// não-direcional (min,max) num hashmap: a primeira vez que vemos a aresta, guardamos
    /// (triângulo, edge local); a segunda vez que aparece, os dois já são vizinhos.
    /// </summary>
    [BurstCompile]
    public struct BuildTriangleAdjacencyJob : IJob
    {
        [ReadOnly] public NativeArray<int3> Triangles;
        public NativeArray<int3> Neighbors;

        public void Execute()
        {
            int n = Triangles.Length;
            for (int i = 0; i < n; i++)
                Neighbors[i] = new int3(-1, -1, -1);

            var edgeMap = new NativeParallelHashMap<long, int2>(n * 3, Allocator.Temp);

            for (int t = 0; t < n; t++)
            {
                int3 tri = Triangles[t];
                for (int e = 0; e < 3; e++)
                {
                    int a = tri[e];
                    int b = tri[(e + 1) % 3];
                    long key = PackEdge(a, b);

                    if (edgeMap.TryGetValue(key, out int2 other))
                    {
                        SetNeighbor(t, e, other.x);
                        SetNeighbor(other.x, other.y, t);
                        edgeMap.Remove(key);
                    }
                    else
                    {
                        edgeMap.TryAdd(key, new int2(t, e));
                    }
                }
            }

            edgeMap.Dispose();
        }

        void SetNeighbor(int tri, int edge, int neighborTri)
        {
            int3 nb = Neighbors[tri];
            nb[edge] = neighborTri;
            Neighbors[tri] = nb;
        }

        static long PackEdge(int a, int b)
        {
            if (a > b) { int tmp = a; a = b; b = tmp; }
            return ((long)a << 32) | (uint)b;
        }
    }
}
