using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>
    /// Insere cada agente ativo numa célula de um grid uniforme (XZ), usado pela
    /// AvoidanceAndMoveJob como broad-phase pra achar vizinhos próximos sem comparar
    /// todos-com-todos (O(n²)). Roda todo frame; o hashmap é limpo (Clear) pelo manager
    /// antes de agendar esse job.
    /// </summary>
    [BurstCompile]
    public struct BuildAgentSpatialHashJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Positions;
        public float CellSize;
        public NativeParallelMultiHashMap<int, int>.ParallelWriter HashWriter;

        public void Execute(int index)
        {
            int2 cell = (int2)math.floor(Positions[index].xz / CellSize);
            int key = AvoidanceAndMoveJob.HashCellKey(cell);
            HashWriter.Add(key, index);
        }
    }
}
