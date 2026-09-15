using System.Collections.Generic;
using Unity.AI.Navigation; // NavMeshLink — vem do pacote com.unity.ai.navigation, NÃO de UnityEngine.AI
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.SceneManagement;

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

        /// <summary>
        /// Área mínima (m², triângulo REAL em 3D — não projetado em XZ) abaixo da qual um
        /// triângulo é descartado do grafo por padrão — ver parâmetro 'minTriangleArea' de
        /// BuildFromUnityNavMesh e o comentário grande lá. Pequeno o bastante pra só pegar
        /// slivers genuinamente degenerados (ruído de voxelização), não geometria real fina
        /// de propósito (rampa estreita, beirada de escada).
        /// </summary>
        public const float DefaultMinTriangleArea = 1e-4f;

        public static NavMeshGraph BuildFromUnityNavMesh(Allocator allocator, float weldEpsilon = DefaultWeldEpsilon, float minTriangleArea = DefaultMinTriangleArea)
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
            int rawTCount = tri.indices.Length / 3;

            var vertices = new NativeArray<float3>(vCount, allocator);
            for (int i = 0; i < vCount; i++)
                vertices[i] = weldedVertices[i];

            // ---- filtro de triângulos-sliver ----
            // Bakes do Recast em geometria de parede complexa/detalhada (relativo ao Voxel
            // Size usado) costumam gerar uma "teia" de triângulos degenerados: faixas
            // extremamente finas e alongadas tentando cobrir uma fronteira que, na resolução
            // do voxel, quase colapsa numa linha. Isso confunde tanto o funil (Funnel.cs,
            // que decide left/right em cima da ÁREA do portal — um portal quase-zero some no
            // ruído de ponto flutuante) quanto o clamp de superfície
            // (AvoidanceAndMoveJob.TestTriangleAndNeighbors, que fica alternando entre
            // slivers vizinhos quase empatados) — na prática aparece como corredor
            // "atravessando" uma parede fina e/ou agente pinado numa borda sem progredir
            // (MovementFaultType.NoProgress). A causa raiz é qualidade de bake (ver README,
            // "Triângulos degenerados (sliver)"), mas descartar esses triângulos do grafo
            // ANTES de montar a adjacência é uma rede de segurança do pacote: eles nunca
            // aparecem pro A*/funil/flow field nem pro clamp. Usa a área REAL em 3D (não
            // projetada em XZ) — uma rampa/degrau estreito de propósito continua com área 3D
            // normal mesmo tendo pegada XZ pequena, então não é descartado por engano.
            //
            // Risco aceito: se um sliver for a ÚNICA ponte entre duas regiões (nenhum
            // triângulo saudável adjacente cobrindo o mesmo trecho), removê-lo pode
            // desconectar essa área — na prática, slivers de voxelização costumam ser
            // "excesso" ao lado de triangulação saudável cobrindo o mesmo espaço físico, não
            // a única rota, mas não há garantia formal disso. minTriangleArea <= 0 desliga o
            // filtro inteiro (comportamento antigo, nenhum triângulo é descartado).
            var keptTriangles = new List<int3>(rawTCount);
            var keptAreas = new List<byte>(rawTCount);
            int slivers = 0;

            for (int t = 0; t < rawTCount; t++)
            {
                int i0 = remap[tri.indices[t * 3 + 0]];
                int i1 = remap[tri.indices[t * 3 + 1]];
                int i2 = remap[tri.indices[t * 3 + 2]];

                if (minTriangleArea > 0f)
                {
                    float3 e1 = vertices[i1] - vertices[i0];
                    float3 e2 = vertices[i2] - vertices[i0];
                    float area = math.length(math.cross(e1, e2)) * 0.5f;
                    if (area < minTriangleArea)
                    {
                        slivers++;
                        continue;
                    }
                }

                keptTriangles.Add(new int3(i0, i1, i2));
                keptAreas.Add((byte)tri.areas[t]);
            }

            if (slivers > 0)
            {
                Debug.LogWarning($"NavMeshGraphBuilder: {slivers} triângulo(s)-sliver (área 3D < " +
                    $"{minTriangleArea:G3}m²) descartado(s) do grafo (de {rawTCount} pra {keptTriangles.Count}) " +
                    "— provável bake com Voxel Size grosso demais pra geometria de parede detalhada nessa " +
                    "área (ver README, 'Triângulos degenerados (sliver)'). Isso é uma rede de segurança, não " +
                    "substitui ajustar o bake se o sintoma persistir (corredor cortando parede, agente sem " +
                    "progresso perto de geometria complexa).");
            }

            int tCount = keptTriangles.Count;
            var triangles = new NativeArray<int3>(tCount, allocator);
            var triArea = new NativeArray<byte>(tCount, allocator);
            var centers = new NativeArray<float3>(tCount, allocator);

            for (int t = 0; t < tCount; t++)
            {
                int3 idx = keptTriangles[t];
                triangles[t] = idx;
                triArea[t] = keptAreas[t];
                centers[t] = (vertices[idx.x] + vertices[idx.y] + vertices[idx.z]) / 3f;
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
        /// Coleta os NavMeshLink (UnityEngine.AI.NavMeshLink — pulos, plataformas, zip-lines
        /// etc.) ativos da cena e monta arestas "virtuais" extras entre o triângulo mais
        /// próximo de cada ponta — usadas por FindPathsBatchJob ALÉM da adjacência normal
        /// (Neighbors). Precisa ser chamado DEPOIS do NavMeshSpatialGrid já estar construído
        /// (usa FindNearestTriangle pra achar em qual triângulo cada ponta do link cai) —
        /// por isso não faz parte de BuildFromUnityNavMesh, e sim de uma chamada separada
        /// do NavMeshJobManager logo depois de montar o grid.
        ///
        /// NavMesh.CalculateTriangulation() não devolve NavMeshLink nenhum — são um tipo de
        /// dado totalmente separado da triangulação baked, então só dá pra descobri-los
        /// escaneando os componentes da cena diretamente (FindObjectsOfType), não tem atalho.
        /// Arrays de saída sempre são criados (tamanho 0 se não houver link nenhum) — quem
        /// consome não precisa checar null, só iterar (loop de tamanho 0 já é um no-op).
        ///
        /// <paramref name="restrictToScene"/> (opcional, null = comportamento padrão): o
        /// `NavMesh` do Unity já é global por natureza (uma única triangulação combinando
        /// TODOS os tiles/cenas carregadas — `FindObjectsOfType` escanear a cena inteira é
        /// consistente com isso, não uma falha de escopo). A situação em que isso pega algo
        /// indesejado é multi-cena ADITIVA com a intenção de várias áreas logicamente
        /// separadas (ex.: instância de masmorra por jogador) — como só sobrevive UM
        /// `NavMeshJobManager` por processo (singleton), ele coletaria `NavMeshLink` de
        /// TODAS as cenas carregadas, mesmo as que "pertenceriam" a outra instância lógica.
        /// Passe a cena do próprio manager aqui pra restringir a coleta só a componentes
        /// dessa cena (ver `NavMeshJobManager.restrictNavMeshLinksToOwnScene`, desligado por
        /// padrão pra preservar o comportamento de antes desse parâmetro existir).
        /// </summary>
        public static void BuildLinks(
            Allocator allocator,
            in NavMeshSpatialGrid grid,
            NativeArray<float3> vertices,
            NativeArray<int3> triangles,
            NativeArray<float> areaCost,
            out NativeArray<float3> linkStart,
            out NativeArray<float3> linkEnd,
            out NativeArray<int> linkFromTriangle,
            out NativeArray<int> linkToTriangle,
            out NativeArray<float> linkCost,
            out NativeArray<byte> linkArea,
            out NativeArray<float> linkWidth,
            Scene? restrictToScene = null)
        {
            var starts = new List<float3>();
            var ends = new List<float3>();
            var froms = new List<int>();
            var tos = new List<int>();
            var costs = new List<float>();
            var areas = new List<byte>();
            var widths = new List<float>();

            var links = Object.FindObjectsOfType<NavMeshLink>();
            for (int i = 0; i < links.Length; i++)
            {
                NavMeshLink link = links[i];
                // FindObjectsOfType() por padrão já não devolve componente de GameObject
                // inativo, mas o componente em si pode estar desabilitado (checkbox) mesmo
                // com o GameObject ativo — checa os dois de qualquer forma, defensivamente.
                if (!link.enabled || !link.gameObject.activeInHierarchy) continue;

                if (restrictToScene.HasValue && link.gameObject.scene != restrictToScene.Value) continue;

                // startPoint/endPoint são em espaço LOCAL do transform do link (documentação
                // do NavMeshLink) — precisa converter pra world antes de achar o triângulo.
                float3 worldStart = link.transform.TransformPoint(link.startPoint);
                float3 worldEnd = link.transform.TransformPoint(link.endPoint);

                int fromTri = NavMeshQueryUtil.FindNearestTriangle(worldStart, grid, vertices, triangles, out _);
                int toTri = NavMeshQueryUtil.FindNearestTriangle(worldEnd, grid, vertices, triangles, out _);
                if (fromTri < 0 || toTri < 0)
                {
                    Debug.LogWarning($"NavMeshGraphBuilder: NavMeshLink '{link.name}' tem uma ponta " +
                        "longe demais de qualquer triângulo do NavMesh (fora da área coberta pelo grid) " +
                        "— ignorado nesta reconstrução do grafo.", link);
                    continue;
                }

                byte area = (byte)link.area;
                // mesma semântica do Unity: costModifier >= 0 é um custo explícito (substitui
                // o custo da área); costModifier < 0 (default -1) usa o custo da área normal.
                float baseCost = link.costModifier >= 0f ? link.costModifier : areaCost[area];
                float cost = math.distance(worldStart, worldEnd) * math.max(baseCost, 0.01f);

                starts.Add(worldStart); ends.Add(worldEnd);
                froms.Add(fromTri); tos.Add(toTri);
                costs.Add(cost); areas.Add(area);
                widths.Add(math.max(0f, link.width));

                if (link.bidirectional)
                {
                    // aresta extra no sentido inverso — mesmo custo (a travessia física é a
                    // mesma nos dois sentidos).
                    starts.Add(worldEnd); ends.Add(worldStart);
                    froms.Add(toTri); tos.Add(fromTri);
                    costs.Add(cost); areas.Add(area);
                    widths.Add(math.max(0f, link.width));
                }
            }

            int n = starts.Count;
            linkStart = new NativeArray<float3>(n, allocator);
            linkEnd = new NativeArray<float3>(n, allocator);
            linkFromTriangle = new NativeArray<int>(n, allocator);
            linkToTriangle = new NativeArray<int>(n, allocator);
            linkCost = new NativeArray<float>(n, allocator);
            linkArea = new NativeArray<byte>(n, allocator);
            linkWidth = new NativeArray<float>(n, allocator);

            for (int i = 0; i < n; i++)
            {
                linkStart[i] = starts[i];
                linkEnd[i] = ends[i];
                linkFromTriangle[i] = froms[i];
                linkToTriangle[i] = tos[i];
                linkCost[i] = costs[i];
                linkArea[i] = areas[i];
                linkWidth[i] = widths[i];
            }
        }

        /// <summary>
        /// Funde vértices coincidentes (dentro de weldEpsilon) num único índice, usando um
        /// grid de quantização (célula = epsilon) como broad-phase — O(n) em vez de O(n²).
        /// Só roda no build (não é hot path), em C# puro na main thread mesmo (não vale a
        /// pena Burst aqui: é lista gerenciada de Vector3[] vinda direto da API do Unity).
        ///
        /// IMPORTANTE: checa a célula do vértice E as 26 vizinhas (janela 3x3x3), não só a
        /// própria — dois vértices genuinamente a MENOS de epsilon um do outro podem cair em
        /// células DIFERENTES se estiverem em lados opostos de uma fronteira de célula (ex.:
        /// epsilon=0.01, um vértice em x=0.0049 e outro em x=0.0051 diferem só 0.0002, mas
        /// arredondam/truncam pra células adjacentes). Sem checar vizinhas, a solda falhava
        /// de um jeito imprevisível — dependia da posição sub-milimétrica exata de cada par,
        /// então funcionava na maioria das costuras/junções e falhava especificamente em
        /// algumas (não só costura de tile — qualquer lugar onde pedaços de malha diferentes
        /// se encontram, como o canto onde múltiplas peças de parede se juntam). A checagem
        /// de distância real (sqrMagnitude &lt;= epsilon²) antes de fundir também evita o
        /// inverso: dois vértices não-relacionados caindo na mesma célula por coincidência.
        /// </summary>
        static void WeldVertices(Vector3[] rawVertices, float epsilon, out List<Vector3> weldedVertices, out int[] remap)
        {
            var map = new Dictionary<QuantizedKey, List<int>>(rawVertices.Length);
            weldedVertices = new List<Vector3>(rawVertices.Length);
            remap = new int[rawVertices.Length];
            float epsilonSq = epsilon * epsilon;

            for (int i = 0; i < rawVertices.Length; i++)
            {
                Vector3 v = rawVertices[i];
                QuantizedKey cell = QuantizedKey.CellOf(v, epsilon);

                int found = -1;
                for (int dx = -1; dx <= 1 && found < 0; dx++)
                for (int dy = -1; dy <= 1 && found < 0; dy++)
                for (int dz = -1; dz <= 1 && found < 0; dz++)
                {
                    if (!map.TryGetValue(cell.Offset(dx, dy, dz), out List<int> bucket)) continue;
                    for (int b = 0; b < bucket.Count; b++)
                    {
                        if ((weldedVertices[bucket[b]] - v).sqrMagnitude <= epsilonSq)
                        {
                            found = bucket[b];
                            break;
                        }
                    }
                }

                if (found >= 0)
                {
                    remap[i] = found;
                    continue;
                }

                int newIndex = weldedVertices.Count;
                weldedVertices.Add(v);
                if (!map.TryGetValue(cell, out List<int> ownBucket))
                    map[cell] = ownBucket = new List<int>(2);
                ownBucket.Add(newIndex);
                remap[i] = newIndex;
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

            /// <summary>Célula de tamanho 'epsilon' contendo 'v' (piso, não arredondamento — grid uniforme padrão pra busca em janela de vizinhos).</summary>
            public static QuantizedKey CellOf(Vector3 v, float epsilon) => new QuantizedKey(
                (long)System.Math.Floor(v.x / (double)epsilon),
                (long)System.Math.Floor(v.y / (double)epsilon),
                (long)System.Math.Floor(v.z / (double)epsilon));

            public QuantizedKey Offset(long dx, long dy, long dz) => new QuantizedKey(x + dx, y + dy, z + dz);

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
