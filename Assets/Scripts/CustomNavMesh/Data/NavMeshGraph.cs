using System;
using Unity.Collections;
using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>
    /// Representação da triangulação do NavMesh do Unity em NativeArrays, pronta pra
    /// ser lida por Jobs/Burst a partir de qualquer thread.
    ///
    /// É construída uma vez (NavMeshGraphBuilder.BuildFromUnityNavMesh) a partir de
    /// NavMesh.CalculateTriangulation() — que é uma chamada de main thread, gerenciada,
    /// e faz uma cópia completa do NavMesh baked. Depois disso, tudo que consulta o
    /// grafo (busca de triângulo, A*, funnel) roda inteiramente em Jobs Burst-compiled,
    /// sem tocar a API gerenciada do Unity.
    /// </summary>
    public struct NavMeshGraph : IDisposable
    {
        /// <summary>Vértices do NavMesh (posições em world space).</summary>
        public NativeArray<float3> Vertices;

        /// <summary>Índices de vértice (x,y,z) de cada triângulo, winding conforme gerado pelo Unity.</summary>
        public NativeArray<int3> Triangles;

        /// <summary>
        /// Triângulo vizinho por aresta: Neighbors[t][e] é o triângulo do outro lado da
        /// aresta (Triangles[t][e], Triangles[t][(e+1)%3]), ou -1 se for uma borda do NavMesh.
        /// </summary>
        public NativeArray<int3> Neighbors;

        /// <summary>Centróide de cada triângulo — cache usado como nó do grafo no A* (heurística/custo de aresta).</summary>
        public NativeArray<float3> Centers;

        /// <summary>Snapshot de NavMesh.GetAreaCost(area) por índice de área (0..31), tirado no momento do build.</summary>
        public NativeArray<float> AreaCost;

        /// <summary>Índice de área (0..31) de cada triângulo.</summary>
        public NativeArray<byte> TriangleArea;

        public int TriangleCount => Triangles.IsCreated ? Triangles.Length : 0;
        public bool IsCreated => Triangles.IsCreated;

        public void Dispose()
        {
            if (Vertices.IsCreated) Vertices.Dispose();
            if (Triangles.IsCreated) Triangles.Dispose();
            if (Neighbors.IsCreated) Neighbors.Dispose();
            if (Centers.IsCreated) Centers.Dispose();
            if (AreaCost.IsCreated) AreaCost.Dispose();
            if (TriangleArea.IsCreated) TriangleArea.Dispose();
        }
    }
}
