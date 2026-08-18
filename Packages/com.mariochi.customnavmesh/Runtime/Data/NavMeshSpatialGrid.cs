using System;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>
    /// Grid uniforme (no plano XZ) que mapeia célula -> lista de triângulos cuja AABB
    /// (em XZ) cruza aquela célula. Usado como broad-phase pra "em qual triângulo está
    /// esse ponto?" sem precisar varrer todos os triângulos do NavMesh a cada consulta.
    ///
    /// É só uma estrutura de dados (NativeParallelMultiHashMap por baixo); a montagem
    /// roda num Job (BuildTriangleGridJob) e as consultas rodam dentro dos Jobs de
    /// pathfinding (NavMeshQueryUtil.FindNearestTriangle).
    /// </summary>
    public struct NavMeshSpatialGrid : IDisposable
    {
        public float2 Min;
        public float CellSize;
        public int2 CellCount;
        public NativeParallelMultiHashMap<int, int> CellToTriangle;

        public bool IsCreated => CellToTriangle.IsCreated;

        public int2 WorldToCell(float2 xz)
        {
            float2 local = (xz - Min) / CellSize;
            return new int2((int)math.floor(local.x), (int)math.floor(local.y));
        }

        /// <summary>
        /// Hash simples de célula -> chave int. Não precisa ser livre de colisão: colisões
        /// só custam candidatos extras nas consultas, nunca resultado incorreto.
        /// </summary>
        public static int CellKey(int2 cell) => cell.x * 92821 + cell.y * 68917;

        public bool IsValidCell(int2 cell) =>
            cell.x >= 0 && cell.y >= 0 && cell.x < CellCount.x && cell.y < CellCount.y;

        public static NavMeshSpatialGrid Build(in NavMeshGraph graph, Allocator allocator, float cellSize)
        {
            var minRef = new NativeReference<float2>(Allocator.TempJob);
            var countRef = new NativeReference<int2>(Allocator.TempJob);

            // capacidade inicial generosa: cada triângulo tende a tocar poucas células,
            // mas triângulos grandes/alongados podem tocar várias.
            int estimatedCapacity = math.max(64, graph.TriangleCount * 4);
            var map = new NativeParallelMultiHashMap<int, int>(estimatedCapacity, allocator);

            new BuildTriangleGridJob
            {
                Vertices = graph.Vertices,
                Triangles = graph.Triangles,
                CellSize = cellSize,
                Min = minRef,
                CellCount = countRef,
                CellToTriangle = map,
            }.Schedule().Complete();

            var grid = new NavMeshSpatialGrid
            {
                Min = minRef.Value,
                CellCount = countRef.Value,
                CellSize = cellSize,
                CellToTriangle = map,
            };

            minRef.Dispose();
            countRef.Dispose();
            return grid;
        }

        public void Dispose()
        {
            if (CellToTriangle.IsCreated) CellToTriangle.Dispose();
        }
    }
}
