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
        public NativeParallelMultiHashMap<long, int> CellToTriangle;

        public bool IsCreated => CellToTriangle.IsCreated;

        public int2 WorldToCell(float2 xz)
        {
            float2 local = (xz - Min) / CellSize;
            return new int2((int)math.floor(local.x), (int)math.floor(local.y));
        }

        /// <summary>
        /// Chave EXATA (sem colisão possível) de célula -> long: empacota cell.x nos 32 bits
        /// altos e cell.y (como uint, sem estender sinal) nos 32 bits baixos. Antes disso, a
        /// chave era um hash int de 32 bits (`cell.x * 92821 + cell.y * 68917`) — em mundos
        /// abertos grandes (coordenadas na casa de ~25.000+ unidades com o cellSize default
        /// de 4), esse produto estourava int.MaxValue, dando wraparound SILENCIOSO (Burst
        /// não lança exceção por overflow) e colisões de célula espúrias que degradavam a
        /// qualidade da busca de triângulo/avoidance sem nenhum sintoma óbvio. Empacotar num
        /// long em vez de tentar "espremer" num hash de 32 bits elimina a classe inteira do
        /// problema — não é uma questão de "hash melhor", é ter bits suficientes pra
        /// representar as duas coordenadas sem perda nenhuma, então nunca há colisão de
        /// verdade (só overhead normal de hashmap, nunca resultado incorreto).
        /// </summary>
        public static long CellKey(int2 cell) => ((long)cell.x << 32) | (uint)cell.y;

        public bool IsValidCell(int2 cell) =>
            cell.x >= 0 && cell.y >= 0 && cell.x < CellCount.x && cell.y < CellCount.y;

        /// <summary>
        /// Comprimento médio de aresta dos triângulos do grafo, PONDERADO POR ÁREA — usado
        /// como base pra calibrar automaticamente o Triangle Grid Cell Size (ver
        /// NavMeshJobManager.autoTriangleGridCellSize). Roda em C# puro na main thread
        /// (não é hot path — só chamado uma vez por RebuildGraph, junto com o resto do
        /// build), então não precisa de Job/Burst pra isso.
        ///
        /// Por que ponderado por área, e não uma média simples por triângulo: um navmesh
        /// com um corredor estreito cheio de triângulos pequenos ao lado de um salão
        /// aberto com poucos triângulos grandes tem uma média SIMPLES dominada pela
        /// CONTAGEM (o corredor tem muito mais triângulos que o salão, mesmo cobrindo bem
        /// menos área) — isso puxava o cell size pra baixo, gerando overhead de grid
        /// desproporcional no salão. Ponderar pela ÁREA de cada triângulo faz a estimativa
        /// refletir a ESCALA DOMINANTE do espaço coberto (quem ocupa mais chão pesa mais),
        /// não quantos triângulos o bake decidiu usar ali. Não é uma solução perfeita — um
        /// grid uniforme sempre é um compromisso único pro mapa inteiro quando a escala
        /// varia muito de região pra região (isso exigiria uma estrutura hierárquica/não-
        /// uniforme pra resolver de verdade) — mas é uma estimativa mais representativa que
        /// a média simples anterior. Se o seu navmesh tiver essa variação extrema e o
        /// automático ainda não performar bem, desligue `Auto Triangle Grid Cell Size` e
        /// calibre manualmente (ver README, "Mapas grandes").
        /// </summary>
        public static float EstimateAverageEdgeLength(in NavMeshGraph graph)
        {
            int count = graph.TriangleCount;
            if (count == 0) return 1f; // sem triângulo nenhum — grid fica vazio de qualquer jeito, valor não importa

            double weightedSum = 0;
            double areaSum = 0;
            for (int t = 0; t < count; t++)
            {
                int3 tri = graph.Triangles[t];
                float3 a = graph.Vertices[tri.x];
                float3 b = graph.Vertices[tri.y];
                float3 c = graph.Vertices[tri.z];

                float avgEdge = (math.distance(a, b) + math.distance(b, c) + math.distance(c, a)) / 3f;
                float area = 0.5f * math.length(math.cross(b - a, c - a));

                weightedSum += (double)avgEdge * area;
                areaSum += area;
            }

            // fallback pra média simples (não ponderada) se a área total ficar ~0 — navmesh
            // degenerado (só triângulos de área zero), caso extremo que não deveria travar
            // a calibração automática mesmo assim.
            if (areaSum < 1e-9)
            {
                double sum = 0;
                for (int t = 0; t < count; t++)
                {
                    int3 tri = graph.Triangles[t];
                    sum += math.distance(graph.Vertices[tri.x], graph.Vertices[tri.y])
                         + math.distance(graph.Vertices[tri.y], graph.Vertices[tri.z])
                         + math.distance(graph.Vertices[tri.z], graph.Vertices[tri.x]);
                }
                return (float)(sum / (count * 3));
            }

            return (float)(weightedSum / areaSum);
        }

        public static NavMeshSpatialGrid Build(in NavMeshGraph graph, Allocator allocator, float cellSize)
        {
            var minRef = new NativeReference<float2>(Allocator.TempJob);
            var countRef = new NativeReference<int2>(Allocator.TempJob);

            // capacidade inicial generosa: cada triângulo tende a tocar poucas células,
            // mas triângulos grandes/alongados podem tocar várias.
            int estimatedCapacity = math.max(64, graph.TriangleCount * 4);
            var map = new NativeParallelMultiHashMap<long, int>(estimatedCapacity, allocator);

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
