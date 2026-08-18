using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.AI;

namespace CustomNavMesh
{
    /// <summary>
    /// Ponte entre a API gerenciada do Unity (NavMesh.CalculateTriangulation, só pode
    /// rodar na main thread) e o NavMeshGraph em NativeArrays consumido pelos Jobs.
    ///
    /// Isso roda só quando o NavMesh é (re)baked — não é hot path por frame. O custo de
    /// varrer a triangulação inteira é pago uma vez aqui; depois disso todo agente
    /// consulta o grafo já pronto, em paralelo, via Burst.
    /// </summary>
    public static class NavMeshGraphBuilder
    {
        /// <summary>
        /// NavMesh.CalculateTriangulation() concatena TODOS os tiles do NavMesh baked
        /// (Recast/Detour divide áreas grandes em tiles), mas não garante índice de vértice
        /// compartilhado na costura entre tiles — cada tile pode gerar seu próprio vértice,
        /// coincidente em posição com o vértice do tile vizinho, mas com índice diferente.
        /// BuildTriangleAdjacencyJob detecta vizinhança por índice de vértice igual, então
        /// sem soldar esses duplicados primeiro, cada tile vira uma "ilha" isolada no grafo
        /// (A* nunca acha caminho atravessando a fronteira de tile, mesmo o NavMesh sendo
        /// visualmente contínuo). Essa distância é a tolerância de solda.
        /// </summary>
        public const float DefaultWeldEpsilon = 0.01f;

        public static NavMeshGraph BuildFromUnityNavMesh(Allocator allocator, float weldEpsilon = DefaultWeldEpsilon)
        {
            NavMeshTriangulation tri = NavMesh.CalculateTriangulation();

            WeldVertices(tri.vertices, weldEpsilon, out List<Vector3> weldedVertices, out int[] remap);

            int weldedAway = tri.vertices.Length - weldedVertices.Count;
            if (weldedAway > 0)
            {
                Debug.Log($"NavMeshGraphBuilder: {weldedAway} vértice(s) duplicado(s) soldado(s) " +
                    $"(de {tri.vertices.Length} pra {weldedVertices.Count}) — provavelmente costuras entre " +
                    "tiles do NavMesh. Se esse número for 0 e ainda assim aparecer NoPath entre agente e " +
                    "destino, o problema é outro (veja README, seção 'Limitações').");
            }

            int vCount = weldedVertices.Count;
            int tCount = tri.indices.Length / 3;

            var vertices = new NativeArray<float3>(vCount, allocator);
            for (int i = 0; i < vCount; i++)
                vertices[i] = weldedVertices[i];

            var triangles = new NativeArray<int3>(tCount, allocator);
            var triArea = new NativeArray<byte>(tCount, allocator);
            var centers = new NativeArray<float3>(tCount, allocator);

            for (int t = 0; t < tCount; t++)
            {
                int i0 = remap[tri.indices[t * 3 + 0]];
                int i1 = remap[tri.indices[t * 3 + 1]];
                int i2 = remap[tri.indices[t * 3 + 2]];

                triangles[t] = new int3(i0, i1, i2);
                triArea[t] = (byte)tri.areas[t];
                centers[t] = (vertices[i0] + vertices[i1] + vertices[i2]) / 3f;
            }

            var areaCost = new NativeArray<float>(NavMeshJobConstants.MaxNavMeshAreas, allocator);
            for (int a = 0; a < NavMeshJobConstants.MaxNavMeshAreas; a++)
                areaCost[a] = (float)NavMesh.GetAreaCost(a);

            var neighbors = new NativeArray<int3>(tCount, allocator);

            var graph = new NavMeshGraph
            {
                Vertices = vertices,
                Triangles = triangles,
                Neighbors = neighbors,
                Centers = centers,
                AreaCost = areaCost,
                TriangleArea = triArea,
            };

            // build de adjacência é rápido (uma passada), mas ainda assim delegamos pro
            // Job Burst-compiled em vez de fazer em C# puro na main thread.
            new BuildTriangleAdjacencyJob
            {
                Triangles = graph.Triangles,
                Neighbors = graph.Neighbors,
            }.Schedule().Complete();

            return graph;
        }

        /// <summary>
        /// Funde vértices coincidentes (dentro de weldEpsilon) num único índice, usando um
        /// grid de quantização como chave de hash — O(n) em vez de O(n²). Só roda no build
        /// (não é hot path), em C# puro na main thread mesmo (não vale a pena Burst aqui:
        /// é lista gerenciada de Vector3[] vinda direto da API do Unity).
        /// </summary>
        static void WeldVertices(Vector3[] rawVertices, float epsilon, out List<Vector3> weldedVertices, out int[] remap)
        {
            var map = new Dictionary<QuantizedKey, int>(rawVertices.Length);
            weldedVertices = new List<Vector3>(rawVertices.Length);
            remap = new int[rawVertices.Length];

            for (int i = 0; i < rawVertices.Length; i++)
            {
                Vector3 v = rawVertices[i];
                var key = QuantizedKey.From(v, epsilon);

                if (map.TryGetValue(key, out int existing))
                {
                    remap[i] = existing;
                }
                else
                {
                    int newIndex = weldedVertices.Count;
                    weldedVertices.Add(v);
                    map[key] = newIndex;
                    remap[i] = newIndex;
                }
            }
        }

        readonly struct QuantizedKey : System.IEquatable<QuantizedKey>
        {
            readonly long x, y, z;

            QuantizedKey(long x, long y, long z)
            {
                this.x = x;
                this.y = y;
                this.z = z;
            }

            public static QuantizedKey From(Vector3 v, float epsilon) => new QuantizedKey(
                (long)System.Math.Round(v.x / (double)epsilon),
                (long)System.Math.Round(v.y / (double)epsilon),
                (long)System.Math.Round(v.z / (double)epsilon));

            public bool Equals(QuantizedKey other) => x == other.x && y == other.y && z == other.z;
            public override bool Equals(object obj) => obj is QuantizedKey k && Equals(k);

            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = 17;
                    hash = hash * 31 + x.GetHashCode();
                    hash = hash * 31 + y.GetHashCode();
                    hash = hash * 31 + z.GetHashCode();
                    return hash;
                }
            }
        }
    }
}
