using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>
    /// Insere cada CustomNavMeshObstacle ativo no MESMO grid espacial usado pelos agentes
    /// (agentSpatialHash), codificando o índice como NEGATIVO (-(obstacleIndex+1)) pra
    /// distinguir de um índice de agente (sempre >= 0) sem precisar de um segundo hashmap —
    /// AvoidanceAndMoveJob decodifica isso ao iterar os vizinhos de uma célula (ver
    /// ComputeOrcaVelocity). Roda em paralelo com BuildAgentSpatialHashJob (mesma
    /// dependência combinada antes do AvoidanceAndMoveJob) — os dois só ESCREVEM no
    /// hashmap (ParallelWriter, chaves prováveis diferentes), nunca leem, então não há
    /// corrida de dados entre eles mesmo rodando ao mesmo tempo.
    /// </summary>
    [BurstCompile]
    public struct BuildObstacleSpatialHashJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> Positions;
        public float CellSize;
        public NativeParallelMultiHashMap<long, int>.ParallelWriter HashWriter;

        public void Execute(int index)
        {
            int2 cell = (int2)math.floor(Positions[index].xz / CellSize);
            long key = AvoidanceAndMoveJob.HashCellKey(cell);
            HashWriter.Add(key, -(index + 1));
        }
    }
}
