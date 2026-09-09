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

        // --- off-mesh links (NavMeshLink do Unity) — arestas "virtuais" extras entre dois
        // triângulos que a triangulação normal não conecta (pulos, plataformas, zip-lines).
        // NavMesh.CalculateTriangulation() não devolve links nenhum; são coletados à parte
        // (NavMeshGraphBuilder.BuildLinks) escaneando os componentes NavMeshLink da cena.
        // Arrays de tamanho 0 (não nulos) quando não há nenhum link — todo código que
        // itera isso vira um no-op automático, sem precisar checar LinkCount==0 antes.
        /// <summary>Ponto de partida (world space) de cada link, na ordem em que a travessia acontece (start→end).</summary>
        public NativeArray<float3> LinkStart;
        /// <summary>Ponto de chegada (world space) correspondente.</summary>
        public NativeArray<float3> LinkEnd;
        /// <summary>Triângulo mais próximo de LinkStart[i].</summary>
        public NativeArray<int> LinkFromTriangle;
        /// <summary>Triângulo mais próximo de LinkEnd[i].</summary>
        public NativeArray<int> LinkToTriangle;
        /// <summary>Custo total da travessia (distância × custo de área, já combinados — ver NavMeshGraphBuilder.BuildLinks).</summary>
        public NativeArray<float> LinkCost;
        /// <summary>Índice de área do link (pra respeitar AreaMask igual uma aresta normal).</summary>
        public NativeArray<byte> LinkArea;
        /// <summary>Largura (NavMeshLink.width) — usada por FindPathsBatchJob.BuildCorridorWithLinks pra dar variação lateral entre agentes cruzando o mesmo link ao mesmo tempo (ver comentário lá).</summary>
        public NativeArray<float> LinkWidth;

        public int TriangleCount => Triangles.IsCreated ? Triangles.Length : 0;
        public int LinkCount => LinkStart.IsCreated ? LinkStart.Length : 0;
        public bool IsCreated => Triangles.IsCreated;

        public void Dispose()
        {
            if (Vertices.IsCreated) Vertices.Dispose();
            if (Triangles.IsCreated) Triangles.Dispose();
            if (Neighbors.IsCreated) Neighbors.Dispose();
            if (Centers.IsCreated) Centers.Dispose();
            if (AreaCost.IsCreated) AreaCost.Dispose();
            if (TriangleArea.IsCreated) TriangleArea.Dispose();
            if (LinkStart.IsCreated) LinkStart.Dispose();
            if (LinkEnd.IsCreated) LinkEnd.Dispose();
            if (LinkFromTriangle.IsCreated) LinkFromTriangle.Dispose();
            if (LinkToTriangle.IsCreated) LinkToTriangle.Dispose();
            if (LinkCost.IsCreated) LinkCost.Dispose();
            if (LinkArea.IsCreated) LinkArea.Dispose();
            if (LinkWidth.IsCreated) LinkWidth.Dispose();
        }
    }
}
