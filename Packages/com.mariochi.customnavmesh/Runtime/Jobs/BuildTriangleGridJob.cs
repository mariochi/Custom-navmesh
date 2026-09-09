using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>
    /// Monta o NavMeshSpatialGrid: calcula os limites (XZ) do NavMesh e insere cada
    /// triângulo em toda célula cuja AABB (em XZ) ele cruza. Roda uma vez por bake.
    /// </summary>
    [BurstCompile]
    public struct BuildTriangleGridJob : IJob
    {
        [ReadOnly] public NativeArray<float3> Vertices;
        [ReadOnly] public NativeArray<int3> Triangles;
        public float CellSize;

        public NativeReference<float2> Min;
        public NativeReference<int2> CellCount;

        /// <summary>Pré-alocado pelo chamador (NavMeshSpatialGrid.Build).</summary>
        public NativeParallelMultiHashMap<long, int> CellToTriangle;

        public void Execute()
        {
            float2 min = new float2(float.MaxValue, float.MaxValue);
            float2 max = new float2(float.MinValue, float.MinValue);

            for (int i = 0; i < Vertices.Length; i++)
            {
                float2 xz = Vertices[i].xz;
                min = math.min(min, xz);
                max = math.max(max, xz);
            }

            // margem de uma célula pra evitar casos de borda exatamente no limite
            min -= CellSize;
            max += CellSize;
            Min.Value = min;

            int2 count = (int2)math.ceil((max - min) / CellSize) + 1;
            CellCount.Value = count;

            for (int t = 0; t < Triangles.Length; t++)
            {
                int3 tri = Triangles[t];
                float2 a = Vertices[tri.x].xz;
                float2 b = Vertices[tri.y].xz;
                float2 c = Vertices[tri.z].xz;

                float2 triMin = math.min(a, math.min(b, c));
                float2 triMax = math.max(a, math.max(b, c));

                int2 cMin = (int2)math.floor((triMin - min) / CellSize);
                int2 cMax = (int2)math.floor((triMax - min) / CellSize);

                for (int cx = cMin.x; cx <= cMax.x; cx++)
                {
                    for (int cy = cMin.y; cy <= cMax.y; cy++)
                    {
                        long key = NavMeshSpatialGrid.CellKey(new int2(cx, cy));
                        CellToTriangle.Add(key, t);
                    }
                }
            }
        }
    }
}
