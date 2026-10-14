using System.Collections.Generic;
using Unity.AI.Navigation; // NavMeshLink — vem do pacote com.unity.ai.navigation, NÃO de UnityEngine.AI
using Unity.Burst;
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

        /// <summary>
        /// Diferença máxima de altura (m) entre um vértice de borda e a aresta de borda do
        /// tile vizinho que passa por ele em XZ pra ainda contar como a mesma costura — ver
        /// RepairTJunctions. Cobre a divergência do detail mesh do Recast entre tiles (dezenas
        /// de cm em rampas/escadas) sem colar bordas genuinamente empilhadas (meio-fio, piso
        /// de cima), que ficam bem mais longe que isso na vertical.
        /// </summary>
        public const float DefaultTJunctionHeightTolerance = 0.3f;

        /// <summary>Folga em XZ (m) pra considerar que um vértice cai sobre uma aresta de borda.</summary>
        const float TJunctionXZTolerance = 0.02f;

        /// <summary>
        /// Resultado de BuildManaged: o grafo em arrays gerenciados comuns, mais as
        /// contagens que LogBuildStats reporta. Existe pra que o trabalho pesado do rebuild
        /// possa rodar FORA da main thread (ver NavMeshJobManager, rebuild em background) —
        /// só NavMesh.CalculateTriangulation() e a cópia final pra NativeArrays (ToNativeGraph)
        /// precisam da main thread.
        /// </summary>
        public sealed class ManagedGraph
        {
            public float3[] Vertices;
            public int3[] Triangles;
            public int3[] Neighbors;
            public float3[] Centers;
            public byte[] TriangleAreas;
            /// <summary>Ver NavMeshSpatialGrid.EstimateAverageEdgeLength (calibração automática do grid).</summary>
            public float AverageEdgeLength;

            public int RawVertexCount;
            public int RawTriangleCount;
            public int WeldedAway;
            public int Slivers;
            public int KeptAfterSliverFilter;
            public int TJunctionsRepaired;
        }

        /// <summary>
        /// Versão síncrona de sempre (main thread do começo ao fim). Equivale a
        /// CalculateTriangulation + BuildManaged + LogBuildStats + ToNativeGraph.
        /// </summary>
        public static NavMeshGraph BuildFromUnityNavMesh(Allocator allocator, float weldEpsilon = DefaultWeldEpsilon,
            float minTriangleArea = DefaultMinTriangleArea, float tJunctionHeightTolerance = DefaultTJunctionHeightTolerance)
        {
            NavMeshTriangulation tri = NavMesh.CalculateTriangulation();
            ManagedGraph built = BuildManaged(tri.vertices, tri.indices, tri.areas, weldEpsilon, minTriangleArea, tJunctionHeightTolerance);
            LogBuildStats(built, minTriangleArea);
            return ToNativeGraph(built, allocator);
        }

        /// <summary>
        /// Todo o processamento da triangulação bruta até o grafo com adjacência: solda de
        /// vértices, filtro de slivers, reparo de T-junctions, centros e vizinhos. Não toca
        /// em nenhuma API do Unity nem em NativeContainer — seguro pra rodar numa thread de
        /// fundo (Task.Run) com os arrays que NavMesh.CalculateTriangulation() devolveu.
        /// </summary>
        public static ManagedGraph BuildManaged(Vector3[] rawVertices, int[] indices, int[] rawAreas,
            float weldEpsilon, float minTriangleArea, float tJunctionHeightTolerance)
        {
            WeldVertices(rawVertices, weldEpsilon, out List<Vector3> weldedVertices, out int[] remap);

            int vCount = weldedVertices.Count;
            int rawTCount = indices.Length / 3;

            var vertices = new float3[vCount];
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
                int i0 = remap[indices[t * 3 + 0]];
                int i1 = remap[indices[t * 3 + 1]];
                int i2 = remap[indices[t * 3 + 2]];

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
                keptAreas.Add((byte)rawAreas[t]);
            }

            int keptAfterSlivers = keptTriangles.Count;
            int splits = tJunctionHeightTolerance > 0f
                ? RepairTJunctions(keptTriangles, keptAreas, vertices, tJunctionHeightTolerance)
                : 0;

            int3[] triangles = keptTriangles.ToArray();
            var centers = new float3[triangles.Length];
            for (int t = 0; t < triangles.Length; t++)
            {
                int3 idx = triangles[t];
                centers[t] = (vertices[idx.x] + vertices[idx.y] + vertices[idx.z]) / 3f;
            }

            return new ManagedGraph
            {
                Vertices = vertices,
                Triangles = triangles,
                Neighbors = BuildAdjacency(triangles),
                Centers = centers,
                TriangleAreas = keptAreas.ToArray(),
                AverageEdgeLength = NavMeshSpatialGrid.EstimateAverageEdgeLength(vertices, triangles),
                RawVertexCount = rawVertices.Length,
                RawTriangleCount = rawTCount,
                WeldedAway = rawVertices.Length - vCount,
                Slivers = slivers,
                KeptAfterSliverFilter = keptAfterSlivers,
                TJunctionsRepaired = splits,
            };
        }

        /// <summary>Logs de diagnóstico do build — main thread (BuildManaged pode rodar fora dela).</summary>
        public static void LogBuildStats(ManagedGraph built, float minTriangleArea)
        {
            if (built.WeldedAway > 0)
            {
                Debug.Log($"NavMeshGraphBuilder: {built.WeldedAway} vértice(s) duplicado(s) soldado(s) " +
                    $"(de {built.RawVertexCount} pra {built.Vertices.Length}) — provavelmente costuras entre " +
                    "tiles do NavMesh. Se esse número for 0 e ainda assim aparecer NoPath entre agente e " +
                    "destino, o problema é outro (veja README, seção 'Limitações').");
            }

            if (built.Slivers > 0)
            {
                Debug.LogWarning($"NavMeshGraphBuilder: {built.Slivers} triângulo(s)-sliver (área 3D < " +
                    $"{minTriangleArea:G3}m²) descartado(s) do grafo (de {built.RawTriangleCount} pra {built.KeptAfterSliverFilter}) " +
                    "— provável bake com Voxel Size grosso demais pra geometria de parede detalhada nessa " +
                    "área (ver README, 'Triângulos degenerados (sliver)'). Isso é uma rede de segurança, não " +
                    "substitui ajustar o bake se o sintoma persistir (corredor cortando parede, agente sem " +
                    "progresso perto de geometria complexa).");
            }

            if (built.TJunctionsRepaired > 0)
            {
                Debug.Log($"NavMeshGraphBuilder: {built.TJunctionsRepaired} T-junction(s) entre tiles reparada(s) " +
                    $"(triângulos agora: {built.Triangles.Length}) — bordas de tile vizinhas que se tocam " +
                    "em XZ mas não compartilham vértice (ver RepairTJunctions).");
            }
        }

        /// <summary>
        /// Copia o resultado de BuildManaged pra NativeArrays (memcpy) e lê o custo de cada
        /// área do NavMesh — main thread.
        /// </summary>
        public static NavMeshGraph ToNativeGraph(ManagedGraph built, Allocator allocator)
        {
            var areaCost = new NativeArray<float>(NavMeshJobConstants.MaxNavMeshAreas, allocator);
            for (int a = 0; a < NavMeshJobConstants.MaxNavMeshAreas; a++)
                areaCost[a] = (float)NavMesh.GetAreaCost(a);

            return new NavMeshGraph
            {
                Vertices = new NativeArray<float3>(built.Vertices, allocator),
                Triangles = new NativeArray<int3>(built.Triangles, allocator),
                Neighbors = new NativeArray<int3>(built.Neighbors, allocator),
                Centers = new NativeArray<float3>(built.Centers, allocator),
                AreaCost = areaCost,
                TriangleArea = new NativeArray<byte>(built.TriangleAreas, allocator),
            };
        }

        /// <summary>
        /// Descobre, pra cada triângulo, o vizinho que compartilha cada uma das 3 arestas.
        /// Uma aresta (a,b) de um triângulo é vista pelo vizinho como (b,a): com uma chave
        /// não-direcional (min,max), a primeira ocorrência guarda (triângulo, aresta local) e
        /// a segunda fecha o par. Em C# gerenciado (não mais um Job Burst) pra poder rodar
        /// dentro de BuildManaged, fora da main thread — Jobs só são agendados nela.
        /// </summary>
        static int3[] BuildAdjacency(int3[] triangles)
        {
            var neighbors = new int3[triangles.Length];
            var open = new Dictionary<long, int>(triangles.Length * 3, MixedLongComparer.Instance);

            for (int t = 0; t < triangles.Length; t++)
            {
                neighbors[t] = new int3(-1, -1, -1);
                int3 tri = triangles[t];
                for (int e = 0; e < 3; e++)
                {
                    long key = PackEdge(tri[e], tri[(e + 1) % 3]);
                    if (open.Remove(key, out int other))
                    {
                        int otherTri = other / 3;
                        int3 nb = neighbors[t];
                        nb[e] = otherTri;
                        neighbors[t] = nb;
                        int3 otherNb = neighbors[otherTri];
                        otherNb[other % 3] = t;
                        neighbors[otherTri] = otherNb;
                    }
                    else
                    {
                        open[key] = t * 3 + e;
                    }
                }
            }

            return neighbors;
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

            var activeLinks = new List<NavMeshLink>();
            var links = Object.FindObjectsOfType<NavMeshLink>();
            for (int i = 0; i < links.Length; i++)
            {
                NavMeshLink link = links[i];
                // FindObjectsOfType() por padrão já não devolve componente de GameObject
                // inativo, mas o componente em si pode estar desabilitado (checkbox) mesmo
                // com o GameObject ativo — checa os dois de qualquer forma, defensivamente.
                if (!link.enabled || !link.gameObject.activeInHierarchy) continue;

                if (restrictToScene.HasValue && link.gameObject.scene != restrictToScene.Value) continue;
                activeLinks.Add(link);
            }

            // triângulo de cada ponta resolvido num Job Burst paralelo — chamar
            // FindNearestTriangle aqui em C# gerenciado custava ~165ms pra 64 links no mapa do
            // jogo, na main thread, a cada rebuild (a maior parte da travada da troca de grafo).
            var endpoints = new NativeArray<float3>(activeLinks.Count * 2, Allocator.TempJob);
            var endpointTriangles = new NativeArray<int>(activeLinks.Count * 2, Allocator.TempJob);
            for (int i = 0; i < activeLinks.Count; i++)
            {
                // startPoint/endPoint são em espaço LOCAL do transform do link (documentação
                // do NavMeshLink) — precisa converter pra world antes de achar o triângulo.
                NavMeshLink link = activeLinks[i];
                endpoints[i * 2] = link.transform.TransformPoint(link.startPoint);
                endpoints[i * 2 + 1] = link.transform.TransformPoint(link.endPoint);
            }
            new FindNearestTrianglesJob
            {
                Points = endpoints,
                Grid = grid,
                Vertices = vertices,
                Triangles = triangles,
                Results = endpointTriangles,
            }.Schedule(endpoints.Length, 8).Complete();

            for (int i = 0; i < activeLinks.Count; i++)
            {
                NavMeshLink link = activeLinks[i];
                float3 worldStart = endpoints[i * 2];
                float3 worldEnd = endpoints[i * 2 + 1];
                int fromTri = endpointTriangles[i * 2];
                int toTri = endpointTriangles[i * 2 + 1];
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

            endpoints.Dispose();
            endpointTriangles.Dispose();

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

        [BurstCompile]
        struct FindNearestTrianglesJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> Points;
            [ReadOnly] public NavMeshSpatialGrid Grid;
            [ReadOnly] public NativeArray<float3> Vertices;
            [ReadOnly] public NativeArray<int3> Triangles;
            [WriteOnly] public NativeArray<int> Results;

            public void Execute(int index) =>
                Results[index] = NavMeshQueryUtil.FindNearestTriangle(Points[index], Grid, Vertices, Triangles, out _);
        }

        /// <summary>
        /// Repara T-junctions nas costuras entre tiles. Dois tiles vizinhos nem sempre
        /// subdividem a borda comum nos mesmos pontos: um lado pode ter um vértice no meio de
        /// uma aresta que, do outro lado, é uma aresta única — e, como o detail mesh do Recast
        /// amostra a altura por tile, esse vértice ainda pode ficar alguns cm acima/abaixo da
        /// aresta do vizinho (visto no jogo: 17cm no meio de uma escada). A solda de vértices
        /// não pega isso (não há vértice correspondente do outro lado), e a adjacência por
        /// índice (BuildTriangleAdjacencyJob) não liga os dois tiles ali — o A* só achava o
        /// outro lado por outro caminho (ou um NavMeshLink de mão única), devolvendo BestEffort
        /// pra destinos que o NavMesh nativo alcança normalmente.
        ///
        /// Correção: pra cada aresta de BORDA (usada por um triângulo só), procura vértices de
        /// borda de outros triângulos que caem sobre ela em XZ (dentro de
        /// TJunctionXZTolerance) e na mesma altura dentro de 'heightTolerance'; o triângulo é
        /// dividido nesses pontos — (a,b,c) com pontos p1..pk em a→b vira (a,p1,c), (p1,p2,c),
        /// ..., (pk,b,c), mesmo winding. Depois disso os índices passam a ser compartilhados e a
        /// adjacência/funil funcionam sem nenhum caso especial. Repete em passadas (uma aresta
        /// por triângulo por passada) até não haver mais nada a dividir.
        /// </summary>
        /// <returns>Número de arestas divididas.</returns>
        static int RepairTJunctions(List<int3> triangles, List<byte> areas, float3[] vertices, float heightTolerance)
        {
            const int MaxPasses = 8;
            int totalSplits = 0;

            // Custo: proporcional a (nº de triângulos + comprimento total das bordas / passo
            // de rasterização) por passada. A 1ª versão varria a CAIXA ENVOLVENTE de cada
            // aresta de borda no grid de vértices (centenas de células por aresta diagonal
            // longa) com o hash padrão de long (x ^ z, colide muito): ~12s por RebuildGraph
            // no mapa do jogo, travando a partida a cada portão/barreira que mudava a malha.
            for (int pass = 0; pass < MaxPasses; pass++)
            {
                var edgeUse = new Dictionary<long, int>(triangles.Count * 3, MixedLongComparer.Instance);
                for (int t = 0; t < triangles.Count; t++)
                {
                    int3 tri = triangles[t];
                    for (int e = 0; e < 3; e++)
                    {
                        long key = PackEdge(tri[e], tri[(e + 1) % 3]);
                        edgeUse.TryGetValue(key, out int n);
                        edgeUse[key] = n + 1;
                    }
                }

                // arestas de borda (t*3+e) rasterizadas no grid XZ, e o conjunto de vértices de borda
                var edgeCells = new Dictionary<long, List<int>>(MixedLongComparer.Instance);
                var boundaryVertices = new HashSet<int>();
                for (int t = 0; t < triangles.Count; t++)
                {
                    int3 tri = triangles[t];
                    for (int e = 0; e < 3; e++)
                    {
                        int a = tri[e], b = tri[(e + 1) % 3];
                        if (edgeUse[PackEdge(a, b)] != 1) continue;
                        boundaryVertices.Add(a);
                        boundaryVertices.Add(b);
                        RasterizeEdge(vertices[a].xz, vertices[b].xz, t * 3 + e, edgeCells);
                    }
                }

                // cada vértice de borda testa só as arestas que passam pela célula dele
                var hitsByEdge = new Dictionary<int, List<(float s, int v)>>();
                foreach (int v in boundaryVertices)
                {
                    float3 p = vertices[v];
                    if (!edgeCells.TryGetValue(CellKeyXZ(p.xz), out List<int> edges)) continue;
                    for (int i = 0; i < edges.Count; i++)
                    {
                        int edgeId = edges[i];
                        int3 tri = triangles[edgeId / 3];
                        int e = edgeId % 3;
                        int a = tri[e], b = tri[(e + 1) % 3], c = tri[(e + 2) % 3];
                        if (v == a || v == b || v == c) continue;
                        if (!IsOnEdge(p, vertices[a], vertices[b], heightTolerance, out float s)) continue;
                        if (!hitsByEdge.TryGetValue(edgeId, out List<(float s, int v)> hits))
                            hitsByEdge[edgeId] = hits = new List<(float s, int v)>(2);
                        if (!hits.Contains((s, v))) hits.Add((s, v)); // mesma aresta em 2 células vizinhas
                    }
                }

                // aplica: uma aresta por triângulo por passada (as outras na próxima)
                int splitsThisPass = 0;
                var splitTriangles = new HashSet<int>();
                foreach (KeyValuePair<int, List<(float s, int v)>> pair in hitsByEdge)
                {
                    int t = pair.Key / 3;
                    if (!splitTriangles.Add(t)) continue;
                    int e = pair.Key % 3;
                    int3 tri = triangles[t];
                    int a = tri[e], b = tri[(e + 1) % 3], c = tri[(e + 2) % 3];

                    List<(float s, int v)> hits = pair.Value;
                    hits.Sort((x, y) => x.s.CompareTo(y.s));
                    byte area = areas[t];
                    int previous = a;
                    for (int h = 0; h < hits.Count; h++)
                    {
                        int3 piece = new int3(previous, hits[h].v, c);
                        if (h == 0) triangles[t] = piece;
                        else { triangles.Add(piece); areas.Add(area); }
                        previous = hits[h].v;
                    }
                    triangles.Add(new int3(previous, b, c));
                    areas.Add(area);
                    splitsThisPass++;
                }

                totalSplits += splitsThisPass;
                if (splitsThisPass == 0) break;
            }

            return totalSplits;
        }

        /// <summary>Célula do grid de RepairTJunctions (metros).</summary>
        const float TJunctionCellSize = 1f;

        /// <summary>
        /// Registra 'edgeId' em toda célula que o segmento a→b atravessa, alargado por
        /// TJunctionXZTolerance — amostrado a cada 1/4 de célula, com as células vizinhas
        /// incluídas quando a amostra está a menos da tolerância da fronteira.
        /// </summary>
        static void RasterizeEdge(float2 a, float2 b, int edgeId, Dictionary<long, List<int>> edgeCells)
        {
            float length = math.distance(a, b);
            int steps = math.max(1, (int)math.ceil(length / (TJunctionCellSize * 0.25f)));
            long lastKey = long.MinValue;
            for (int i = 0; i <= steps; i++)
            {
                float2 p = math.lerp(a, b, i / (float)steps);
                float2 local = p / TJunctionCellSize - math.floor(p / TJunctionCellSize);
                int2 cell = (int2)math.floor(p / TJunctionCellSize);
                int2 lo = new int2(local.x * TJunctionCellSize < TJunctionXZTolerance ? -1 : 0, local.y * TJunctionCellSize < TJunctionXZTolerance ? -1 : 0);
                int2 hi = new int2((1f - local.x) * TJunctionCellSize < TJunctionXZTolerance ? 1 : 0, (1f - local.y) * TJunctionCellSize < TJunctionXZTolerance ? 1 : 0);
                for (int dx = lo.x; dx <= hi.x; dx++)
                for (int dz = lo.y; dz <= hi.y; dz++)
                {
                    long key = PackCell(cell.x + dx, cell.y + dz);
                    if (dx == 0 && dz == 0)
                    {
                        if (key == lastKey) continue; // amostras consecutivas na mesma célula
                        lastKey = key;
                    }
                    if (!edgeCells.TryGetValue(key, out List<int> list))
                        edgeCells[key] = list = new List<int>(4);
                    if (list.Count == 0 || list[list.Count - 1] != edgeId)
                        list.Add(edgeId);
                }
            }
        }

        /// <summary>'p' cai sobre a aresta a→b (XZ dentro da tolerância, fora das pontas, altura dentro de heightTolerance)?</summary>
        static bool IsOnEdge(float3 p, float3 pa, float3 pb, float heightTolerance, out float s)
        {
            s = 0f;
            float2 ab = pb.xz - pa.xz;
            float lenSq = math.lengthsq(ab);
            if (lenSq < 1e-8f) return false;
            float len = math.sqrt(lenSq);
            float endMargin = TJunctionXZTolerance / len; // não aceita pontos colados nas pontas
            float2 ap = p.xz - pa.xz;
            s = math.dot(ap, ab) / lenSq;
            if (s <= endMargin || s >= 1f - endMargin) return false;
            if (math.abs(ap.x * ab.y - ap.y * ab.x) / len > TJunctionXZTolerance) return false;
            return math.abs(p.y - math.lerp(pa.y, pb.y, s)) <= heightTolerance;
        }

        static long PackEdge(int a, int b)
        {
            if (a > b) { int tmp = a; a = b; b = tmp; }
            return ((long)a << 32) | (uint)b;
        }

        static long PackCell(int x, int z) => ((long)x << 32) | (uint)z;

        static long CellKeyXZ(float2 p) =>
            PackCell((int)math.floor(p.x / TJunctionCellSize), (int)math.floor(p.y / TJunctionCellSize));

        /// <summary>
        /// Hash de long com mistura de bits. O GetHashCode padrão de long é (alto ^ baixo):
        /// pra chaves empacotadas como (x &lt;&lt; 32 | z), células/arestas com x ^ z iguais caem
        /// no mesmo bucket — milhares de colisões num grid centrado na origem.
        /// </summary>
        sealed class MixedLongComparer : IEqualityComparer<long>
        {
            public static readonly MixedLongComparer Instance = new MixedLongComparer();
            public bool Equals(long x, long y) => x == y;
            public int GetHashCode(long value) => (int)(((ulong)value * 0x9E3779B97F4A7C15UL) >> 32);
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
