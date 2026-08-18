using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>
    /// Job batelado de pathfinding: cada iteração (Execute(index)) resolve UM PathRequest
    /// inteiro — localizar triângulo de início/fim, A* sobre o grafo de triângulos e
    /// funnel/string-pulling pra gerar o corredor final de waypoints. Tudo Burst-compiled,
    /// rodando em paralelo entre os requests do frame nos worker threads.
    ///
    /// É esse job que substitui o NavMesh.CalculatePath do Unity (que é síncrono e preso
    /// à main thread): aqui N agentes pedindo caminho no mesmo frame são resolvidos em
    /// paralelo, e o resultado só é sincronizado de volta quando o NavMeshJobManager dá
    /// Complete() no JobHandle.
    ///
    /// INVARIANTE IMPORTANTE: os arrays de saída (CorridorOut/CorridorLengthOut/StatusOut)
    /// são indexados por PathRequest.AgentIndex, não pelo índice do job (`index`). Como o
    /// Unity não consegue provar estaticamente que dois requests do mesmo batch nunca têm
    /// o mesmo AgentIndex, os campos de saída usam [NativeDisableParallelForRestriction].
    /// Quem monta o batch (NavMeshJobManager) PRECISA garantir que cada agente apareça no
    /// máximo uma vez por batch — do contrário há uma corrida de dados real.
    /// </summary>
    [BurstCompile]
    public struct FindPathsBatchJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<PathRequest> Requests;

        [ReadOnly] public NativeArray<float3> Vertices;
        [ReadOnly] public NativeArray<int3> Triangles;
        [ReadOnly] public NativeArray<int3> Neighbors;
        [ReadOnly] public NativeArray<float3> Centers;
        [ReadOnly] public NativeArray<float> AreaCost;
        [ReadOnly] public NativeArray<byte> TriangleArea;
        [ReadOnly] public NavMeshSpatialGrid Grid;

        [NativeDisableParallelForRestriction] public NativeArray<float3> CorridorOut;    // flat: AgentIndex * MaxCorridorPoints + i
        [NativeDisableParallelForRestriction] public NativeArray<int> CorridorLengthOut; // por AgentIndex
        [NativeDisableParallelForRestriction] public NativeArray<byte> StatusOut;        // por AgentIndex

        public void Execute(int index)
        {
            PathRequest req = Requests[index];
            int outBase = req.AgentIndex * NavMeshJobConstants.MaxCorridorPoints;

            int startTri = NavMeshQueryUtil.FindNearestTriangle(req.Start, Grid, Vertices, Triangles, out float3 startPoint);
            int endTri = NavMeshQueryUtil.FindNearestTriangle(req.End, Grid, Vertices, Triangles, out float3 endPoint);

            if (startTri < 0 || endTri < 0)
            {
                StatusOut[req.AgentIndex] = (byte)PathStatus.Invalid;
                CorridorLengthOut[req.AgentIndex] = 0;
                return;
            }

            if (startTri == endTri)
            {
                CorridorOut[outBase + 0] = startPoint;
                CorridorOut[outBase + 1] = endPoint;
                CorridorLengthOut[req.AgentIndex] = 2;
                StatusOut[req.AgentIndex] = (byte)PathStatus.Success;
                return;
            }

            int n = Triangles.Length;
            var gScore = new NativeArray<float>(n, Allocator.Temp);
            var cameFrom = new NativeArray<int>(n, Allocator.Temp);
            var closed = new NativeArray<bool>(n, Allocator.Temp);
            for (int i = 0; i < n; i++)
            {
                gScore[i] = float.MaxValue;
                cameFrom[i] = -1;
            }

            var heap = new NativeMinHeap(64, Allocator.Temp);
            gScore[startTri] = 0f;
            heap.Push(startTri, math.distance(Centers[startTri], Centers[endTri]));

            bool found = false;

            while (heap.Count > 0)
            {
                int current = heap.Pop();
                if (closed[current]) continue; // lazy deletion de entradas duplicadas no heap
                closed[current] = true;

                if (current == endTri)
                {
                    found = true;
                    break;
                }

                int3 nbs = Neighbors[current];
                for (int e = 0; e < 3; e++)
                {
                    int nb = nbs[e];
                    if (nb < 0 || closed[nb]) continue;
                    if (!NavMeshQueryUtil.IsAreaAllowed(TriangleArea[nb], req.AreaMask)) continue;

                    float stepCost = math.distance(Centers[current], Centers[nb]) * math.max(AreaCost[TriangleArea[nb]], 0.01f);
                    float tentativeG = gScore[current] + stepCost;

                    if (tentativeG < gScore[nb])
                    {
                        cameFrom[nb] = current;
                        gScore[nb] = tentativeG;
                        float f = tentativeG + math.distance(Centers[nb], Centers[endTri]);
                        heap.Push(nb, f);
                    }
                }
            }

            if (!found)
            {
                heap.Dispose();
                gScore.Dispose();
                cameFrom.Dispose();
                closed.Dispose();

                StatusOut[req.AgentIndex] = (byte)PathStatus.NoPath;
                CorridorLengthOut[req.AgentIndex] = 0;
                return;
            }

            var triPath = new NativeList<int>(16, Allocator.Temp);
            int cur = endTri;
            while (cur != -1)
            {
                triPath.Add(cur);
                cur = cameFrom[cur];
            }
            ReverseInPlace(triPath);

            var waypoints = new NativeList<float3>(16, Allocator.Temp);
            Funnel.BuildCorridor(startPoint, endPoint, triPath, Triangles, Vertices, waypoints);

            int count = math.min(waypoints.Length, NavMeshJobConstants.MaxCorridorPoints);
            for (int i = 0; i < count; i++)
                CorridorOut[outBase + i] = waypoints[i];

            CorridorLengthOut[req.AgentIndex] = count;
            StatusOut[req.AgentIndex] = (byte)(waypoints.Length > NavMeshJobConstants.MaxCorridorPoints
                ? PathStatus.PartialCorridor
                : PathStatus.Success);

            heap.Dispose();
            gScore.Dispose();
            cameFrom.Dispose();
            closed.Dispose();
            triPath.Dispose();
            waypoints.Dispose();
        }

        static void ReverseInPlace(NativeList<int> list)
        {
            int i = 0, j = list.Length - 1;
            while (i < j)
            {
                int tmp = list[i];
                list[i] = list[j];
                list[j] = tmp;
                i++;
                j--;
            }
        }
    }
}
