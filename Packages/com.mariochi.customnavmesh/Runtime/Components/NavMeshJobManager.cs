using System.Collections;
using System.Collections.Generic;
using Unity.AI.Navigation; // NavMeshLink — vem do pacote com.unity.ai.navigation, NÃO de UnityEngine.AI
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Jobs;

namespace CustomNavMesh
{
    /// <summary>
    /// Orquestrador central do sistema. Um único NavMeshJobManager por cena (singleton).
    ///
    /// Responsabilidades:
    ///  - Construir o NavMeshGraph a partir de NavMesh.CalculateTriangulation() (RebuildGraph,
    ///    chame de novo sempre que rebake o NavMesh em runtime).
    ///  - Manter arrays densos (NativeArray, capacidade fixa definida no Inspector) com o
    ///    estado de cada agente registrado, em paralelo com um TransformAccessArray.
    ///  - A cada frame: bater os pedidos de repath pendentes (limitados por
    ///    maxPathRequestsPerFrame, pra não gerar picos de custo), agendar o
    ///    FindPathsBatchJob, o hash espacial de agentes e o AvoidanceAndMoveJob encadeados
    ///    por dependência, e sincronizar tudo no fim do frame.
    ///
    /// Pipeline: os NativeArrays por trás da API (Velocity, IsOnNavMesh, Status,
    /// SetVelocityOverride, SetPaused, Radius/Height/MaxSpeed etc.) só podem ser lidos ou
    /// escritos com segurança quando NENHUM Job está em voo. Pra dar essa garantia pra
    /// qualquer script de gameplay (que roda no Update() dele, execution order padrão),
    /// Schedule() e Complete() do Job acontecem juntos, na MESMA chamada de LateUpdate()
    /// (ordem -100, depois de TODO Update() da cena já ter rodado) — ver comentário em
    /// LateUpdate() pra detalhes e o contrato de quando é seguro chamar a API do agente.
    /// O ganho de multithreading sobre o NavMeshAgent padrão do Unity continua vindo de
    /// resolver todos os agentes em PARALELO entre si nos worker threads (não de sobrepor
    /// com outro código do frame, benefício do qual abrimos mão em troca de correção).
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class NavMeshJobManager : MonoBehaviour
    {
        [Header("Capacidade")]
        [Tooltip("Número máximo de agentes registrados simultaneamente. Os NativeArrays são alocados com esse tamanho fixo no Awake.")]
        [SerializeField] int agentCapacity = 256;
        [Tooltip("Quantos pedidos de repath (novo destino) são processados por frame, no máximo. Limita picos de custo quando muitos agentes pedem caminho no mesmo frame.")]
        [SerializeField] int maxPathRequestsPerFrame = 32;
        [Tooltip("Número máximo de CustomNavMeshObstacle registrados simultaneamente (obstáculos " +
            "dinâmicos pra avoidance — ver CustomNavMeshObstacle). Pool separado da capacidade de " +
            "agentes; a maioria dos jogos precisa de bem menos obstáculos dinâmicos que agentes.")]
        [SerializeField] int obstacleCapacity = 32;
        [Tooltip("Restringe a coleta de NavMeshLink (ver seção 'NavMeshLink' do README) só " +
            "aos componentes que pertencem à MESMA cena deste NavMeshJobManager. Desligado " +
            "(default) preserva o comportamento normal — o NavMesh do Unity já é global por " +
            "natureza, então escanear a cena inteira é consistente com o resto do pipeline. " +
            "Só ligue isso se você tem múltiplas áreas logicamente separadas carregadas " +
            "aditivamente (ex.: instância de masmorra por jogador) e precisa que cada uma " +
            "só enxergue os links da sua própria cena — como só sobrevive UM " +
            "NavMeshJobManager por processo (singleton), sem isso ele coletaria NavMeshLink " +
            "de TODAS as cenas carregadas, mesmo as que 'pertenceriam' a outra instância.")]
        [SerializeField] bool restrictNavMeshLinksToOwnScene = false;

        [Header("Grid do NavMesh (broad-phase de triângulo)")]
        [Tooltip("Se ligado (default), 'Triangle Grid Cell Size' é ignorado e recalculado " +
            "automaticamente a cada RebuildGraph() como (comprimento médio de aresta dos " +
            "triângulos do NavMesh) × 'Auto Cell Size Multiplier' — a regra prática que o " +
            "README já recomendava calibrar manualmente ('comece em 2-4x o tamanho médio de " +
            "aresta'), só que sem precisar abrir o Profiler/olhar o Debug.Log e ajustar à " +
            "mão pra cada navmesh novo. Cobre automaticamente o caso de mapas muito grandes " +
            "(regra de bolso baseada só no tamanho do mapa dava um grid fino demais quando o " +
            "navmesh era uma triangulação aberta/grosseira, e grosso demais quando era densa) " +
            "e reage sozinho a rebakes que mudam a densidade de triângulos. Desligue e ajuste " +
            "'Triangle Grid Cell Size' manualmente se o automático não performar bem pro seu " +
            "caso específico (ex.: navmesh com triângulos de tamanho muito desigual, onde uma " +
            "média simples não representa bem a distribuição real).")]
        [SerializeField] bool autoTriangleGridCellSize = true;
        [Tooltip("Multiplicador aplicado sobre o comprimento médio de aresta quando 'Auto " +
            "Triangle Grid Cell Size' está ligado. Valores maiores = células maiores = menos " +
            "overhead de grid mas mais candidatos por consulta; menores = mais preciso mas " +
            "mais células. 3 é um meio-termo razoável dentro da faixa 2-4x recomendada.")]
        [SerializeField] float autoCellSizeMultiplier = 3f;
        [Tooltip("Usado como Triangle Grid Cell Size SÓ quando 'Auto Triangle Grid Cell Size' " +
            "está desligado — calibração 100% manual, comportamento de antes desse campo existir.")]
        [SerializeField] float triangleGridCellSize = 4f;
        [Tooltip("Distância pra considerar dois vértices da triangulação como o mesmo ponto (solda) " +
            "ao montar o grafo de adjacência. Necessário porque NavMesh.CalculateTriangulation() não " +
            "garante índice de vértice compartilhado na costura entre tiles do NavMesh (comum em mapas " +
            "grandes) — sem soldar, cada tile vira uma ilha isolada e o A* nunca acha caminho entre eles.")]
        [SerializeField] float vertexWeldEpsilon = NavMeshGraphBuilder.DefaultWeldEpsilon;
        [Tooltip("Margem de histerese (metros) que um triângulo vizinho precisa vencer POR, em " +
            "distância, pra substituir o triângulo em cache no clamp de superfície de cada agente " +
            "(ClampToNavMesh). Sem isso, em trechos com triângulos pequenos e muito próximos entre si " +
            "— principalmente degraus de escada — o 'triângulo mais próximo' fica alternando entre dois " +
            "candidatos quase empatados a cada frame só por ruído de sub-milímetro na posição; cada " +
            "troca reprojeta a posição clampada (sobretudo o Y) discretamente, e como a direção do " +
            "próximo frame é calculada a partir dessa posição, isso aparece como zigue-zague bem " +
            "localizado nas bordas dos degraus. Valor pequeno o bastante pra não atrapalhar uma " +
            "transição real entre triângulos (que muda a posição por muito mais que isso conforme o " +
            "agente anda) — se o zigue-zague em escadas persistir, suba um pouco; se agentes parecerem " +
            "'grudar' um frame a mais que deveriam ao mudar de triângulo, abaixe.")]
        [SerializeField] float triangleStickyMargin = 0.02f;
        [Tooltip("Nº máximo de saltos de adjacência que ClampToNavMesh explora (via BFS por " +
            "NavNeighbors) antes de recorrer à busca irrestrita no grid espacial inteiro, " +
            "quando o cache local (triângulo atual + 3 vizinhos diretos) não é suficiente " +
            "pra reconectar a posição do agente nesse frame (empurrão de avoidance grande, " +
            "cruzamento de vários triângulos pequenos de uma vez). Por caminhar só por " +
            "adjacência REAL da malha, essa busca NUNCA pode 'atravessar' uma parede/vão " +
            "fino — diferente da busca irrestrita final (FindNearestTriangle), que aceita o " +
            "triângulo mais próximo em linha reta mesmo que seja do lado errado de um " +
            "obstáculo fino (o único ponto do clamp com esse risco; ela só entra em jogo " +
            "depois que esta camada também falhar). Default 6 cobre movimento bem mais " +
            "rápido que o normal sem custo perceptível (só roda quando o cache falha, não " +
            "todo frame); 0 desliga essa camada (comportamento antigo).")]
        [SerializeField] int wallSafeBfsHops = 6;
        [Tooltip("Distância (metros) além da qual o Transform de um agente é considerado " +
            "'movido por fora' da posição escrita pelo AvoidanceAndMoveJob no frame anterior " +
            "(física de knockback, root motion de animação, cutscene, etc. mexendo direto no " +
            "Transform em vez de usar SetVelocityOverride). Quando isso acontece, a posição " +
            "externa é ADOTADA como novo ponto de partida da simulação (reclampada no NavMesh) " +
            "em vez de silenciosamente sobrescrita no próximo frame. Precisa ser maior que o " +
            "ruído de ponto flutuante da própria escrita do Job (ínfimo); default (5cm) já " +
            "cobre isso com folga sem disparar por qualquer tremor sub-milimétrico.")]
        [SerializeField] float externalMoveTolerance = 0.05f;
        [Tooltip("Master switch pro mecanismo de NotifyNavMeshChanged(): se desligado, chamadas a " +
            "esse método são ignoradas, e o polling de Stale Detection Interval também não age " +
            "(útil pra desligar tudo de uma vez em debug/profiling). A via recomendada continua " +
            "sendo o jogo chamar NotifyNavMeshChanged() explicitamente quando um NavMeshObstacle/ " +
            "portão muda (reage no mesmo frame); Stale Detection Interval é só uma rede de " +
            "segurança pro caso de esquecimento (ver README, 'Rebuild quando o NavMesh muda').")]
        [SerializeField] bool autoRebuildOnNavMeshChange = true;
        [Tooltip("Janela de silêncio (segundos) depois da última chamada a NotifyNavMeshChanged() antes " +
            "de reconstruir o grafo — evita reconstruir uma vez por chamada quando várias mudanças " +
            "acontecem em sequência rápida (ex.: vários NavMeshObstacle entrando em cena juntos).")]
        [SerializeField] float autoRebuildDebounce = 0.25f;
        [Tooltip("Rede de segurança pra quando o jogo esquece de chamar NotifyNavMeshChanged() " +
            "explicitamente: a cada X segundos, compara uma assinatura barata do estado atual " +
            "(nº de vértices/índices/soma de coordenadas da triangulação, E nº/soma de posição " +
            "dos NavMeshLink ativos na cena — os dois são checados porque um NavMeshLink " +
            "mudando não altera a triangulação em si) contra a última usada pro grafo em uso — " +
            "se DIFERIR, chama NotifyNavMeshChanged() sozinho e loga um aviso. " +
            "NÃO substitui a chamada explícita (que reage no mesmo frame; isso aqui só pega o " +
            "esquecimento, com atraso de até este intervalo) e só roda enquanto Auto Rebuild On " +
            "NavMesh Change estiver ligado. Diferente de um candidato óbvio e descartado " +
            "(NavMesh.onPreUpdate, que dispara a cada tick do subsistema de navegação MESMO " +
            "sem nada ter mudado — ver comentário em NotifyNavMeshChanged), este mecanismo só " +
            "chama CalculateTriangulation() esporadicamente (no ritmo deste intervalo, não a " +
            "cada tick) e só age quando a assinatura realmente muda, então não corre o risco de " +
            "reempurrar um debounce que nunca fecha. 0 ou negativo desliga esse polling " +
            "(comportamento 100% manual, como antes).")]
        [SerializeField] float staleDetectionInterval = 2f;

        [Header("Avoidance")]
        [Tooltip("Tamanho de célula do grid usado pra achar agentes/obstáculos próximos (broad-phase " +
            "de avoidance) — não confundir com Triangle Grid Cell Size, que é sobre triângulos do " +
            "NavMesh. AvoidanceAndMoveJob escala automaticamente a JANELA de busca (quantas células " +
            "em volta são varridas) pra sempre cobrir o raio de detecção efetivo de cada agente " +
            "(ver Neighbor Query Radius), então não precisa recalibrar isso manualmente quando algum " +
            "agente tem Radius bem maior que o normal.")]
        [SerializeField] float neighborCellSize = 2f;
        [Tooltip("Raio (metros) de busca de vizinhos pra avoidance, valor global (por-agente via " +
            "SetAvoidanceOverride). ESCALADO PRA CIMA automaticamente por agente — nunca pra baixo — " +
            "pra garantir um mínimo de 2×Radius desse agente: sem isso, dois agentes com Radius bem " +
            "maior que este valor (ex.: Radius 5 cada, raio combinado 10, com Neighbor Query Radius " +
            "no default de 4) podiam estar profundamente sobrepostos fisicamente e MESMO ASSIM nunca " +
            "virarem candidato um do outro — uma 'parede invisível' de detecção. Pra agentes de " +
            "Radius normal (bem menor que este valor), o escalonamento não muda nada (fica exatamente " +
            "neste valor, sem overhead extra).")]
        [SerializeField] float neighborQueryRadius = 4f;
        [SerializeField] float avoidanceTimeHorizon = 2f;
        [SerializeField] float defaultWaypointReachDistance = 0.3f;
        [Tooltip("Diferença de altura (Y) acima da qual dois agentes não se enxergam pra avoidance, " +
            "mesmo próximos em XZ — sem isso, um agente em cima de uma muralha/ponte e outro embaixo " +
            "dela se desviam um do outro como se estivessem no mesmo plano. Ajuste pra cobrir a altura " +
            "típica de um agente (evita ignorar quem está genuinamente ao lado numa rampa suave) sem " +
            "cobrir a altura de estruturas que devem separar tropas (muralhas, pontes, plataformas).")]
        [SerializeField] float verticalAvoidanceRange = 2f;
        [Tooltip("Limite de variação de velocidade, como múltiplo de MaxSpeed por segundo (ex.: 8 = " +
            "sai do repouso até a velocidade máxima em ~1/8s). Suaviza mudanças bruscas de direção " +
            "(cruzar de triângulo no flow field, convergência de grupo perto do destino) em vez de " +
            "deixar a velocidade saltar direto pro valor desejado todo frame. 0 desliga a suavização.")]
        [SerializeField] float steeringAccelerationFactor = 10f;

        [Header("Flow Field (grupos)")]
        [Tooltip("Nº máximo de flow fields simultâneos (um por chamada de MoveGroupWithFlowField ainda " +
            "em uso por pelo menos 1 agente). Cada slot custa TriangleCount * (12+4 bytes) de memória " +
            "persistente. Se estourar, MoveGroupWithFlowField loga erro e não faz nada.")]
        [SerializeField] int maxFlowFields = 8;
        [Tooltip("Distância-até-o-alvo (ao longo do campo, não em linha reta) abaixo da qual o agente " +
            "é promovido do flow field pro pipeline individual (SetDestination, A* + funnel) rumo " +
            "ao seu ponto exato de chegada — a reta final passa a respeitar a malha em vez de seguir " +
            "linha reta ingênua até o ponto de formação.")]
        [SerializeField] float flowFieldArriveDistance = 2f;
        [Tooltip("TESTE DIAGNÓSTICO: se marcado, agentes em modo flow field pulam o avoidance inteiro " +
            "(não consultam vizinhos, seguem só a direção do campo + suavização de velocidade) — pode " +
            "se sobrepor entre si. Serve pra isolar se um zigue-zague residual vem do avoidance.")]
        [SerializeField] bool flowFieldIgnoresAvoidance = false;

        [Header("Debug")]
        [SerializeField] bool drawNavMeshGizmo = false;
        [SerializeField] bool drawCorridorGizmos = false;
        [Tooltip("Desenha uma esfera colorida sobre cada agente com o PathStatus atual: " +
            "cinza = None (nunca pediu path / ainda não processado), verde = Success, " +
            "amarelo = PartialCorridor, vermelho = NoPath, magenta = Invalid (start/end fora do NavMesh), " +
            "ciano = FlowField (seguindo campo de grupo), laranja = BestEffort (destino inalcançável — " +
            "foi levado até o ponto mais perto possível, dentro da mesma ilha do NavMesh).")]
        [SerializeField] bool drawStatusGizmos = false;
        [Tooltip("Desenha uma seta por triângulo com a direção de fluxo do slot escolhido em Flow Field Gizmo Slot.")]
        [SerializeField] bool drawFlowFieldGizmo = false;
        [SerializeField] int flowFieldGizmoSlot = 0;

        public static NavMeshJobManager Instance { get; private set; }

        NavMeshGraph graph;
        NavMeshSpatialGrid triGrid;
        bool graphReady;

        /// <summary>Nº de triângulos do grafo atual. Útil pra calibrar Triangle Grid Cell Size / avaliar custo do A* por request.</summary>
        public int TriangleCount => graph.TriangleCount;

        /// <summary>Nº de agentes registrados no momento.</summary>
        public int ActiveAgentCount => count;

        int capacity;
        int count;

        readonly List<CustomNavMeshAgent> agentComponents = new List<CustomNavMeshAgent>();
        TransformAccessArray transformAccessArray;

        NativeArray<float3> positions;
        NativeArray<float3> outPositions;
        NativeArray<float3> velocities;
        NativeArray<float3> prevVelocities;
        NativeArray<float> radii;
        NativeArray<float> maxSpeeds;
        NativeArray<float> waypointReachDistances;
        NativeArray<float> heights;
        NativeArray<int> corridorCursor;
        NativeArray<int> corridorLength;
        NativeArray<byte> pathStatus;
        NativeArray<int> currentTriangle;
        NativeArray<float3> corridorFlat;
        NativeArray<bool> corridorIsLinkArrival; // mesmo layout de corridorFlat — true[j] = chegar em CorridorFlat[j] é pousar depois de atravessar um NavMeshLink (ver FindPathsBatchJob/AvoidanceAndMoveJob)
        NativeArray<byte> movementFault; // MovementFaultType do frame atual, por agente — diagnóstico (ver AvoidanceAndMoveJob)
        NativeArray<bool> movementFaultLogged; // já logamos esse agente pelo menos uma vez (evita spam no Console)
        NativeArray<bool> paused; // Pause()/Resume() — trava a busca ativa sem descartar corredor/flow field
        NativeArray<bool> ignoreAvoidance; // por agente, permanente até trocar de novo
        NativeArray<float> neighborRadiusOverride; // -1 = usa o global; resetado todo frame em LateUpdate (válido só 1 frame)
        NativeArray<float> timeHorizonOverride; // -1 = usa o global; idem
        NativeArray<float3> velocityOverride; // válido só se hasVelocityOverride[i] — resetado todo frame em LateUpdate
        NativeArray<bool> hasVelocityOverride;

        // --- flow field: por agente (capacidade fixa) + pool de campos achatado (depende de TriangleCount) ---
        NativeArray<int> flowFieldSlot; // -1 = agente no modo corredor
        NativeArray<float3> flowFieldDirections; // flat: slot * graph.TriangleCount + triângulo
        NativeArray<float> flowFieldDistance;
        NativeArray<float3> flowFieldTargetPoints; // por slot (tamanho maxFlowFields) — ponto "bruto" do comando, sem formação
        NativeArray<float3> flowFieldPersonalTarget; // por agente — ponto que ele mira de fato perto do alvo (com offset de formação, se houver)
        FlowFieldMeta[] flowFieldMeta; // bookkeeping gerenciado, pequeno, não precisa ser NativeArray

        NativeParallelMultiHashMap<long, int> agentSpatialHash;

        // --- obstáculos dinâmicos (CustomNavMeshObstacle) — pool separado, sem pathfinding
        // nenhum, só posição/raio/velocidade lidos a cada frame pra virar restrição extra de
        // ORCA (responsabilidade 100% do agente, não-recíproca — ver AvoidanceAndMoveJob).
        int obstacleCapacityInternal;
        int obstacleCount;
        readonly List<CustomNavMeshObstacle> obstacleComponents = new List<CustomNavMeshObstacle>();
        NativeArray<float3> obstaclePositions;
        NativeArray<float3> obstacleVelocities; // estimada por diferença de posição frame a frame — ver RefreshObstacles
        NativeArray<float> obstacleRadii;

        readonly HashSet<int> pendingRepathSet = new HashSet<int>();
        readonly List<int> repathBatchBuffer = new List<int>();

        NativeList<PathRequest> frameRequests;
        JobHandle frameHandle;

        /// <summary>Bookkeeping de um slot do pool de flow fields. Puramente gerenciado (main thread only).</summary>
        struct FlowFieldMeta
        {
            public bool InUse;
            public int RefCount;
            public int TargetTriangle;
        }

        /// <summary>
        /// Todas as instâncias VIVAS (sobrevivendo desde que a destruição automática de
        /// duplicata foi removida — ver Awake()), independente de qual é o 'Instance'
        /// estático padrão. Usado só pela rede de segurança de descarte de memória em
        /// Editor (OnEditorPlayModeStateChanged), que precisa limpar TODAS as instâncias,
        /// não só a default.
        /// </summary>
        static readonly List<NavMeshJobManager> allInstances = new List<NavMeshJobManager>();

        void Awake()
        {
            // Antes, uma segunda instância na cena (ou numa cena carregada aditivamente)
            // era DESTRUÍDA aqui — isso quebrava qualquer setup multi-cena com várias
            // áreas de navmesh logicamente separadas (ex.: instância de masmorra por
            // jogador): só a PRIMEIRA instância sobrevivia, e todo agente/obstáculo de
            // QUALQUER cena carregada se registrava nela — mesmo pertencendo a uma área
            // com coordenadas/grafo completamente diferentes (ver README, "Múltiplas
            // instâncias / multi-cena"). Agora cada instância sobrevive e funciona de
            // forma independente (seu próprio grafo, seus próprios agentes); só a
            // PRIMEIRA continua virando o 'Instance' estático (o alvo default pra quem
            // não aponta um manager explícito via CustomNavMeshAgent.Manager/
            // CustomNavMeshObstacle.Manager) — preserva o comportamento de sempre no caso
            // comum (uma cena, um manager, zero configuração extra).
            if (Instance == null)
            {
                Instance = this;
            }
            else if (Instance != this)
            {
                Debug.Log($"NavMeshJobManager: '{Instance.name}' já é a instância padrão " +
                    $"('Instance' estático) — '{name}' continua funcionando normalmente, mas " +
                    "agentes/obstáculos que não apontarem um manager explícito (ver campo " +
                    "'Manager' em CustomNavMeshAgent/CustomNavMeshObstacle) vão se registrar " +
                    $"em '{Instance.name}', não em '{name}'.", this);
            }

            // guarda contra duplicata: se Awake() rodar de novo pro MESMO objeto sobrevivente
            // sem que OnDestroy() tenha rodado entre uma sessão de Play e outra (mesmo combo
            // raro do item 5 acima), evita empilhar referências repetidas na lista estática.
            if (!allInstances.Contains(this))
                allInstances.Add(this);

            // rede de segurança pra "Reload Domain" + "Reload Scene" ambos DESLIGADOS
            // (Project Settings > Editor > Enter Play Mode Settings, combo raro): nesse
            // modo, o MESMO GameObject/Component sobrevive intacto de uma sessão de Play
            // pra outra (a cena nunca é descartada), mas Awake() continua rodando de novo
            // a cada entrada em Play — é assim que o modo rápido funciona. Se os
            // NativeArrays Allocator.Persistent da sessão ANTERIOR nunca foram dispostos
            // (OnDestroy só roda quando o objeto é de fato destruído, o que não acontece
            // se a cena não é recarregada), alocar por cima de novo vazaria memória nativa
            // a cada ciclo Play/Stop. Detecta isso (algum array ainda IsCreated) e limpa
            // tudo (nativo + bookkeeping gerenciado) antes de alocar de novo.
            if (positions.IsCreated)
                DisposeAllPersistent();

            capacity = math.max(1, agentCapacity);
            AllocatePersistent();
        }

#if UNITY_EDITOR
        static NavMeshJobManager()
        {
            UnityEditor.EditorApplication.playModeStateChanged += OnEditorPlayModeStateChanged;
        }

        /// <summary>
        /// Segunda camada da mesma rede de segurança (ver comentário em Awake()): força o
        /// descarte da memória nativa no momento em que o Editor COMEÇA a sair do Play
        /// Mode, sem depender de MonoBehaviour.OnDestroy() rodar (que pode não rodar no
        /// combo "Reload Domain"+"Reload Scene" desligados, se o GameObject não chegar a
        /// ser realmente destruído). Idempotente — DisposeAllPersistent() só mexe em
        /// arrays que ainda estão IsCreated, então rodar duas vezes (aqui e de novo via um
        /// OnDestroy() normal, se ele rodar) não tem custo nem risco. Só existe em builds
        /// de Editor (compilado fora com #if), zero custo/footprint em builds de jogador.
        /// Itera TODAS as instâncias vivas (allInstances), não só 'Instance' — desde que a
        /// destruição automática de duplicata foi removida (ver Awake()), pode haver mais
        /// de uma instância simultânea, e todas precisam da mesma rede de segurança.
        /// </summary>
        static void OnEditorPlayModeStateChanged(UnityEditor.PlayModeStateChange state)
        {
            if (state != UnityEditor.PlayModeStateChange.ExitingPlayMode) return;

            for (int i = 0; i < allInstances.Count; i++)
            {
                if (allInstances[i] != null)
                    allInstances[i].DisposeAllPersistent();
            }
        }
#endif

        Coroutine autoRebuildCoroutine;
        float autoRebuildDeadline;
        Coroutine staleDetectionCoroutine;
        NavMeshSignature lastSignature;

        /// <summary>
        /// Assinatura BARATA do estado atual — não é um hash criptográfico nem uma
        /// comparação vértice-a-vértice (isso custaria o mesmo que reconstruir o grafo
        /// inteiro), só o suficiente pra pegar a esmagadora maioria das mudanças reais
        /// (obstáculo apareceu/sumiu, portão fechou, rebake mudou a malha) sem custar mais
        /// que a própria chamada a CalculateTriangulation() que já precisa ser feita de
        /// qualquer jeito pra checar.
        ///
        /// Cobre DUAS fontes de mudança independentes: a triangulação em si (Vertex/Index
        /// Count + CoordSum) E os NavMeshLink da cena (LinkCount + LinkCoordSum) — as duas
        /// são necessárias porque um NavMeshLink sendo adicionado/movido/(des)ativado/
        /// removido em runtime NÃO muda a triangulação (NavMesh.CalculateTriangulation()
        /// não inclui link nenhum, ver NavMeshGraphBuilder.BuildLinks), então uma
        /// assinatura baseada só na triangulação NUNCA detectaria essa mudança —
        /// StalenessWatcher ficaria estruturalmente cego a qualquer alteração de link,
        /// exigindo NotifyNavMeshChanged() manual sempre nesse caso específico (o que não
        /// estava documentado antes desta assinatura cobrir os dois).
        /// </summary>
        readonly struct NavMeshSignature
        {
            public readonly int VertexCount;
            public readonly int IndexCount;
            public readonly float CoordSum;
            public readonly int LinkCount;
            public readonly float LinkCoordSum;

            public NavMeshSignature(int vertexCount, int indexCount, float coordSum, int linkCount, float linkCoordSum)
            {
                VertexCount = vertexCount;
                IndexCount = indexCount;
                CoordSum = coordSum;
                LinkCount = linkCount;
                LinkCoordSum = linkCoordSum;
            }

            public bool DiffersFrom(NavMeshSignature other)
            {
                if (VertexCount != other.VertexCount || IndexCount != other.IndexCount) return true;
                if (LinkCount != other.LinkCount) return true;

                // tolerância relativa, não fixa: CoordSum/LinkCoordSum são somas de
                // coordenadas (podem chegar a milhões num mapa grande), e soma em float
                // perde precisão absoluta conforme o valor cresce — um limiar fixo pequeno
                // (ex.: 0.001) dispararia falso-positivo por ruído de arredondamento
                // sozinho em mapas grandes. Falso-positivo aqui é barato (só agenda um
                // rebuild a mais); falso-negativo é o risco real, então o piso absoluto
                // (0.001) ainda cobre mapas pequenos onde 1e-5 relativo seria pequeno
                // demais pra notar.
                float tolerance = math.max(0.001f, math.abs(CoordSum) * 1e-5f);
                if (math.abs(CoordSum - other.CoordSum) > tolerance) return true;

                float linkTolerance = math.max(0.001f, math.abs(LinkCoordSum) * 1e-5f);
                return math.abs(LinkCoordSum - other.LinkCoordSum) > linkTolerance;
            }
        }

        static NavMeshSignature ComputeSignature(NavMeshTriangulation tri)
        {
            double sum = 0;
            var verts = tri.vertices;
            for (int i = 0; i < verts.Length; i++)
                sum += verts[i].x + verts[i].y + verts[i].z;

            // NavMeshLink não é parte da triangulação — precisa ser escaneado à parte (ver
            // comentário em NavMeshSignature). Barato: mesmo escaneamento que BuildLinks já
            // faz, só que aqui é só contagem + soma de posição, sem lookup de triângulo.
            var links = Object.FindObjectsOfType<NavMeshLink>();
            int linkCount = 0;
            double linkSum = 0;
            for (int i = 0; i < links.Length; i++)
            {
                NavMeshLink link = links[i];
                if (!link.enabled || !link.gameObject.activeInHierarchy) continue;

                linkCount++;
                float3 s = link.transform.TransformPoint(link.startPoint);
                float3 e = link.transform.TransformPoint(link.endPoint);
                linkSum += s.x + s.y + s.z + e.x + e.y + e.z;
            }

            return new NavMeshSignature(verts.Length, tri.indices.Length, (float)sum, linkCount, (float)linkSum);
        }

        void OnDisable()
        {
            // se o GameObject for desativado com a corrotina de debounce em voo, o Unity mata a
            // corrotina sozinho mas NÃO zera nossa referência — sem isso, NotifyNavMeshChanged()
            // acharia (errado) que ainda tem uma corrotina rodando depois de reativado, e o
            // debounce ficaria travado pra sempre (nunca mais chamando StartCoroutine de novo).
            autoRebuildCoroutine = null;
            staleDetectionCoroutine = null; // mesmo motivo, pro polling de StalenessWatcher
        }

        /// <summary>
        /// Chame isso quando SOUBER que o NavMesh mudou em runtime — um NavMeshObstacle ligou/
        /// desligou carving, um NavMeshLink foi ativado, um portão fechou, etc. Agenda um
        /// RebuildGraph() + RepathAllAgents() debounced (várias chamadas em sequência rápida
        /// viram uma única reconstrução, ver Auto Rebuild Debounce). Ignorado se Auto Rebuild On
        /// NavMesh Change estiver desligado no inspector.
        ///
        /// NÃO existe uma forma confiável de detectar "o NavMesh mudou" automaticamente: o
        /// candidato óbvio, NavMesh.onPreUpdate, dispara a cada tick do subsistema de navegação
        /// do Unity — ou seja, também dispara quando NADA mudou — então não dá pra diferenciar
        /// "mudou" de "só rodou o tick" só com esse evento (uma versão anterior deste método
        /// tentava usar isso com debounce e ficava presa: o prazo nunca vencia, porque o evento
        /// reempurrava o debounce de novo antes da janela fechar). Por isso o gatilho é explícito
        /// — chamar isso continua sendo a via RECOMENDADA (reage no mesmo frame). Como rede de
        /// segurança pra quando o jogo esquece de chamar, StalenessWatcher (ver Stale Detection
        /// Interval) chama este método sozinho se detectar, por polling periódico barato (não
        /// por evento a cada tick — não sofre do problema acima), que a triangulação OU os
        /// NavMeshLink ativos da cena mudaram (as duas coisas são checadas separadamente —
        /// um NavMeshLink mudando não altera a triangulação em si, ver NavMeshSignature).
        /// </summary>
        public void NotifyNavMeshChanged()
        {
            if (!autoRebuildOnNavMeshChange) return;

            // debounce: cada chamada empurra o prazo pra frente; a corrotina só executa o
            // rebuild de fato depois de 'autoRebuildDebounce' segundos SEM nenhuma chamada nova
            // — várias mudanças em sequência rápida (vários obstáculos entrando juntos) viram
            // uma única reconstrução em vez de uma por chamada.
            autoRebuildDeadline = Time.time + autoRebuildDebounce;
            if (autoRebuildCoroutine == null)
                autoRebuildCoroutine = StartCoroutine(AutoRebuildDebounced());
        }

        IEnumerator AutoRebuildDebounced()
        {
            while (Time.time < autoRebuildDeadline)
                yield return null;

            autoRebuildCoroutine = null;

            // fecha o ciclo completo: grafo novo (topologia atualizada) + força quem já tinha
            // corredor calculado a recalcular em cima da malha nova (RebuildGraph sozinho não
            // faz isso — só invalida os índices de triângulo, não repropõe destino a ninguém).
            RebuildGraph();
            RepathAllAgents();
        }

        /// <summary>
        /// Rede de segurança: a cada 'staleDetectionInterval' segundos, compara a assinatura
        /// barata da triangulação atual contra a do grafo em uso (ver NavMeshSignature) e, se
        /// diferir, chama NotifyNavMeshChanged() sozinho — pega o caso do jogo esquecer de
        /// notificar explicitamente (NavMeshObstacle, carving, rebake feito por outro sistema
        /// sem saber que este pacote existe). Roda em polling periódico, não por evento a cada
        /// tick, então não sofre do problema do NavMesh.onPreUpdate documentado acima (esse
        /// dispara mesmo sem nada mudar; isso aqui só age quando a assinatura de fato muda).
        /// Desligado (staleDetectionInterval <= 0) por padrão de custo zero fora do intervalo:
        /// só chama CalculateTriangulation() uma vez por tick deste laço, nunca por frame.
        /// </summary>
        IEnumerator StalenessWatcher()
        {
            while (true)
            {
                yield return new WaitForSeconds(math.max(0.1f, staleDetectionInterval));

                if (!autoRebuildOnNavMeshChange) continue; // desligado no meio do jogo — não gasta CalculateTriangulation à toa
                if (!graphReady) continue; // ainda não teve um primeiro RebuildGraph bem-sucedido pra comparar contra

                var signature = ComputeSignature(NavMesh.CalculateTriangulation());
                if (signature.DiffersFrom(lastSignature))
                {
                    Debug.Log("NavMeshJobManager: detectei automaticamente uma mudança na triangulação do NavMesh " +
                        "sem NotifyNavMeshChanged() ter sido chamado (Stale Detection Interval) — agendando rebuild. " +
                        "Se isso disparar com frequência, considere chamar NotifyNavMeshChanged() explicitamente no " +
                        "código que muda o NavMesh (mais responsivo que esperar o próximo tick deste polling).", this);
                    NotifyNavMeshChanged();
                }
            }
        }

        void Start()
        {
            // NÃO construir o grafo no Awake: com [DefaultExecutionOrder(-100)] o Awake deste
            // manager roda antes do OnEnable de outros scripts da cena — incluindo o
            // NavMeshSurface, que só registra os dados baked em NavMesh.AddNavMeshData()
            // no próprio OnEnable. O Unity garante que TODO Awake/OnEnable da cena termina
            // antes de QUALQUER Start(), independente de execution order — então esperar até
            // aqui garante que NavMesh.CalculateTriangulation() já vai enxergar a malha.
            if (Instance == this)
            {
                RebuildGraph();

                if (staleDetectionInterval > 0f)
                    staleDetectionCoroutine = StartCoroutine(StalenessWatcher());
            }
        }

        void AllocatePersistent()
        {
            transformAccessArray = new TransformAccessArray(capacity);
            positions = new NativeArray<float3>(capacity, Allocator.Persistent);
            outPositions = new NativeArray<float3>(capacity, Allocator.Persistent);
            velocities = new NativeArray<float3>(capacity, Allocator.Persistent);
            prevVelocities = new NativeArray<float3>(capacity, Allocator.Persistent);
            radii = new NativeArray<float>(capacity, Allocator.Persistent);
            maxSpeeds = new NativeArray<float>(capacity, Allocator.Persistent);
            waypointReachDistances = new NativeArray<float>(capacity, Allocator.Persistent);
            heights = new NativeArray<float>(capacity, Allocator.Persistent);
            corridorCursor = new NativeArray<int>(capacity, Allocator.Persistent);
            corridorLength = new NativeArray<int>(capacity, Allocator.Persistent);
            pathStatus = new NativeArray<byte>(capacity, Allocator.Persistent);
            currentTriangle = new NativeArray<int>(capacity, Allocator.Persistent);
            corridorFlat = new NativeArray<float3>(capacity * NavMeshJobConstants.MaxCorridorPoints, Allocator.Persistent);
            corridorIsLinkArrival = new NativeArray<bool>(capacity * NavMeshJobConstants.MaxCorridorPoints, Allocator.Persistent);
            movementFault = new NativeArray<byte>(capacity, Allocator.Persistent);
            movementFaultLogged = new NativeArray<bool>(capacity, Allocator.Persistent);
            paused = new NativeArray<bool>(capacity, Allocator.Persistent);
            ignoreAvoidance = new NativeArray<bool>(capacity, Allocator.Persistent);
            neighborRadiusOverride = new NativeArray<float>(capacity, Allocator.Persistent);
            timeHorizonOverride = new NativeArray<float>(capacity, Allocator.Persistent);
            velocityOverride = new NativeArray<float3>(capacity, Allocator.Persistent);
            hasVelocityOverride = new NativeArray<bool>(capacity, Allocator.Persistent);
            flowFieldSlot = new NativeArray<int>(capacity, Allocator.Persistent);
            flowFieldPersonalTarget = new NativeArray<float3>(capacity, Allocator.Persistent);

            for (int i = 0; i < capacity; i++)
            {
                currentTriangle[i] = -1;
                flowFieldSlot[i] = -1;
                neighborRadiusOverride[i] = -1f;
                timeHorizonOverride[i] = -1f;
            }

            int ffCapacity = math.max(1, maxFlowFields);
            flowFieldMeta = new FlowFieldMeta[ffCapacity];
            flowFieldTargetPoints = new NativeArray<float3>(ffCapacity, Allocator.Persistent);
            // flowFieldDirections/flowFieldDistance dependem de graph.TriangleCount, que só existe depois
            // do primeiro RebuildGraph() (chamado no Start) — alocados/realocados lá, não aqui.

            obstacleCapacityInternal = math.max(1, obstacleCapacity);
            obstaclePositions = new NativeArray<float3>(obstacleCapacityInternal, Allocator.Persistent);
            obstacleVelocities = new NativeArray<float3>(obstacleCapacityInternal, Allocator.Persistent);
            obstacleRadii = new NativeArray<float>(obstacleCapacityInternal, Allocator.Persistent);

            // capacidade do hashmap cobre agentes E obstáculos — os dois compartilham o mesmo
            // NativeParallelMultiHashMap (obstáculo usa índice negativo, ver BuildObstacleSpatialHashJob).
            agentSpatialHash = new NativeParallelMultiHashMap<long, int>(math.max(64, capacity + obstacleCapacityInternal), Allocator.Persistent);
        }

        /// <summary>
        /// Reconstrói o grafo a partir do NavMesh atualmente baked. Chame depois de
        /// NavMeshSurface.BuildNavMesh() (ou qualquer outro rebake em runtime). Seguro
        /// de chamar a qualquer momento — espera o trabalho em voo terminar antes de trocar o grafo.
        /// </summary>
        public void RebuildGraph()
        {
            frameHandle.Complete();

            if (graph.IsCreated) graph.Dispose();
            if (triGrid.IsCreated) triGrid.Dispose();

            graph = NavMeshGraphBuilder.BuildFromUnityNavMesh(Allocator.Persistent, vertexWeldEpsilon);

            // calibração automática do Triangle Grid Cell Size (ver tooltip do campo): pega o
            // comprimento médio de aresta dos triângulos DESTE grafo e aplica o multiplicador —
            // mesma regra prática que o README já recomendava calibrar manualmente, mas recalculada
            // sozinha a cada rebuild (reage a rebakes que mudam a densidade de triângulos, e cobre
            // mapas de qualquer tamanho sem precisar de ajuste manual por cena).
            float effectiveCellSize = triangleGridCellSize;
            if (autoTriangleGridCellSize && graph.TriangleCount > 0)
            {
                float avgEdge = NavMeshSpatialGrid.EstimateAverageEdgeLength(graph);
                effectiveCellSize = math.max(0.01f, avgEdge * autoCellSizeMultiplier);
            }

            triGrid = NavMeshSpatialGrid.Build(graph, Allocator.Persistent, effectiveCellSize);
            graphReady = graph.TriangleCount > 0;

            // off-mesh links (NavMeshLink da cena) — precisa do triGrid já pronto (usa
            // FindNearestTriangle) então só roda aqui, não dentro de BuildFromUnityNavMesh.
            // Sempre aloca os arrays (mesmo vazios) pra manter Dispose() simétrico e todo
            // consumidor podendo iterar sem checar null.
            if (graphReady)
            {
                NavMeshGraphBuilder.BuildLinks(Allocator.Persistent, triGrid, graph.Vertices, graph.Triangles, graph.AreaCost,
                    out NativeArray<float3> linkStart, out NativeArray<float3> linkEnd,
                    out NativeArray<int> linkFromTriangle, out NativeArray<int> linkToTriangle,
                    out NativeArray<float> linkCost, out NativeArray<byte> linkArea,
                    out NativeArray<float> linkWidth,
                    restrictNavMeshLinksToOwnScene ? gameObject.scene : (UnityEngine.SceneManagement.Scene?)null);
                graph.LinkStart = linkStart;
                graph.LinkEnd = linkEnd;
                graph.LinkFromTriangle = linkFromTriangle;
                graph.LinkToTriangle = linkToTriangle;
                graph.LinkCost = linkCost;
                graph.LinkArea = linkArea;
                graph.LinkWidth = linkWidth;

                if (graph.LinkCount > 0)
                    Debug.Log($"NavMeshJobManager: {graph.LinkCount} NavMeshLink coletado(s) da cena e conectado(s) ao grafo.", this);
            }
            else
            {
                graph.LinkStart = new NativeArray<float3>(0, Allocator.Persistent);
                graph.LinkEnd = new NativeArray<float3>(0, Allocator.Persistent);
                graph.LinkFromTriangle = new NativeArray<int>(0, Allocator.Persistent);
                graph.LinkToTriangle = new NativeArray<int>(0, Allocator.Persistent);
                graph.LinkCost = new NativeArray<float>(0, Allocator.Persistent);
                graph.LinkArea = new NativeArray<byte>(0, Allocator.Persistent);
                graph.LinkWidth = new NativeArray<float>(0, Allocator.Persistent);
            }

            // assinatura da triangulação que ACABOU de ser usada pra montar o grafo — referência
            // pra StalenessWatcher comparar depois. Custa uma 2ª chamada a CalculateTriangulation()
            // (a 1ª foi dentro de BuildFromUnityNavMesh, que não expõe a triangulação bruta pra
            // fora), aceitável aqui porque RebuildGraph já é uma operação pesada e pouco frequente.
            lastSignature = ComputeSignature(NavMesh.CalculateTriangulation());

            // buffers de flow field são dimensionados por TriangleCount — precisam ser realocados
            // toda vez que o grafo muda, e os campos antigos ficam inválidos (índices de triângulo
            // não valem mais nada), então libera todo mundo que estava usando algum.
            if (flowFieldDirections.IsCreated) flowFieldDirections.Dispose();
            if (flowFieldDistance.IsCreated) flowFieldDistance.Dispose();

            int ffCapacity = math.max(1, maxFlowFields);
            int flatSize = math.max(1, ffCapacity * graph.TriangleCount);
            flowFieldDirections = new NativeArray<float3>(flatSize, Allocator.Persistent);
            flowFieldDistance = new NativeArray<float>(flatSize, Allocator.Persistent);

            for (int i = 0; i < flowFieldMeta.Length; i++)
                flowFieldMeta[i] = default; // InUse = false

            for (int i = 0; i < count; i++)
                flowFieldSlot[i] = -1; // volta todo agente pro modo corredor até pedir um novo grupo

            if (!graphReady)
            {
                Debug.LogWarning("NavMeshJobManager: NavMesh.CalculateTriangulation() não retornou nenhum triângulo. " +
                    "Faça o bake do NavMesh (Window > AI > Navigation, ou um NavMeshSurface.BuildNavMesh()) antes de pedir caminhos.", this);
            }
            else
            {
                // dado pra calibrar 'Triangle Grid Cell Size': se avgCellsTouched ficar muito
                // acima de ~2-3, o grid está fino demais (cellSize pequeno pra área do mapa);
                // se ficar perto de 1 mas o grid tiver poucas células no total, pode ir menor.
                float coveredX = triGrid.CellCount.x * triGrid.CellSize;
                float coveredZ = triGrid.CellCount.y * triGrid.CellSize;
                string cellSizeSource = autoTriangleGridCellSize ? "auto" : "manual";
                Debug.Log($"NavMeshJobManager: grafo reconstruído — {graph.TriangleCount} triângulos, " +
                    $"grid {triGrid.CellCount.x}x{triGrid.CellCount.y} células de {triGrid.CellSize:F2}u ({cellSizeSource}) " +
                    $"cobrindo ~{coveredX:F0}x{coveredZ:F0}u. Se path/registro de agente parecer lento, " +
                    $"ajuste 'Auto Cell Size Multiplier' ou desligue 'Auto Triangle Grid Cell Size' e calibre " +
                    $"'Triangle Grid Cell Size' manualmente (ver README, seção 'Mapas grandes').", this);
            }

            for (int i = 0; i < count; i++)
                currentTriangle[i] = -1; // força nova busca de triângulo pros agentes já registrados
        }

        /// <summary>
        /// Força todo agente com destino ativo (modo corredor) a recalcular o caminho — chame
        /// depois de RebuildGraph() se a topologia mudou (portão fechou, obstáculo apareceu) e o
        /// corredor antigo pode ter virado inválido/subótimo sem que ninguém tenha detectado isso
        /// sozinho. Agentes sem destino (nunca chamaram SetDestination) são ignorados — não tem
        /// pra onde recalcular. Agentes em flow field também são ignorados aqui de propósito: um
        /// RebuildGraph() já tira todo mundo do flow field automaticamente (volta pro modo
        /// corredor); se o grupo ainda precisa se mover, chame MoveGroupWithFlowField de novo.
        /// </summary>
        public void RepathAllAgents()
        {
            for (int i = 0; i < count; i++)
                RequestRepathIfPossible(i);
        }

        /// <summary>
        /// Versão barata de RepathAllAgents: só recalcula quem está a até 'radius' unidades de
        /// 'point' (posição atual do agente, não o destino) — útil quando você sabe exatamente
        /// onde a malha mudou (ex.: um portão específico) e não quer pagar o custo de recalcular
        /// o exército inteiro.
        /// </summary>
        public void RepathAgentsNear(Vector3 point, float radius)
        {
            float3 p = point;
            float r2 = radius * radius;
            for (int i = 0; i < count; i++)
            {
                if (math.distancesq(positions[i], p) <= r2)
                    RequestRepathIfPossible(i);
            }
        }

        void RequestRepathIfPossible(int i)
        {
            if (flowFieldSlot[i] >= 0) return; // flow field é responsabilidade de quem chamou MoveGroupWithFlowField, não mexe aqui

            var agent = agentComponents[i];
            if (agent == null || !agent.HasDestination) return; // nada pra recalcular

            // reforça a MESMA flag suja que SetDestination usa — reaproveita CollectRepathRequests
            // pra enfileirar o pedido, sem precisar de nenhum mecanismo novo.
            agent.SetDestination((Vector3)agent.Destination);
        }

        void OnDestroy()
        {
            DisposeAllPersistent();
            allInstances.Remove(this);
            if (Instance == this) Instance = null;
        }

        /// <summary>
        /// Dispõe TODOS os NativeArrays/NativeContainers persistentes e reseta o
        /// bookkeeping gerenciado (listas/conjuntos/contadores) pro estado "recém-Awake,
        /// nada registrado ainda". Extraído de OnDestroy() pra também poder ser chamado
        /// como rede de segurança a partir de Awake()/do hook de Editor (ver comentários
        /// lá) quando NativeArrays de uma sessão de Play anterior sobrevivem sem terem
        /// sido dispostos. Idempotente: cada dispose é guardado por IsCreated, então
        /// chamar isso mais de uma vez em sequência (ex.: pelo hook de Editor e de novo
        /// pelo OnDestroy() normal logo em seguida) é seguro e não tem custo real na
        /// segunda vez.
        /// </summary>
        void DisposeAllPersistent()
        {
            frameHandle.Complete();

            if (frameRequests.IsCreated) frameRequests.Dispose();

            if (transformAccessArray.isCreated) transformAccessArray.Dispose();
            if (positions.IsCreated) positions.Dispose();
            if (outPositions.IsCreated) outPositions.Dispose();
            if (velocities.IsCreated) velocities.Dispose();
            if (prevVelocities.IsCreated) prevVelocities.Dispose();
            if (radii.IsCreated) radii.Dispose();
            if (maxSpeeds.IsCreated) maxSpeeds.Dispose();
            if (waypointReachDistances.IsCreated) waypointReachDistances.Dispose();
            if (heights.IsCreated) heights.Dispose();
            if (corridorCursor.IsCreated) corridorCursor.Dispose();
            if (corridorLength.IsCreated) corridorLength.Dispose();
            if (pathStatus.IsCreated) pathStatus.Dispose();
            if (currentTriangle.IsCreated) currentTriangle.Dispose();
            if (corridorFlat.IsCreated) corridorFlat.Dispose();
            if (corridorIsLinkArrival.IsCreated) corridorIsLinkArrival.Dispose();
            if (movementFault.IsCreated) movementFault.Dispose();
            if (movementFaultLogged.IsCreated) movementFaultLogged.Dispose();
            if (paused.IsCreated) paused.Dispose();
            if (ignoreAvoidance.IsCreated) ignoreAvoidance.Dispose();
            if (neighborRadiusOverride.IsCreated) neighborRadiusOverride.Dispose();
            if (timeHorizonOverride.IsCreated) timeHorizonOverride.Dispose();
            if (velocityOverride.IsCreated) velocityOverride.Dispose();
            if (hasVelocityOverride.IsCreated) hasVelocityOverride.Dispose();
            if (flowFieldSlot.IsCreated) flowFieldSlot.Dispose();
            if (flowFieldPersonalTarget.IsCreated) flowFieldPersonalTarget.Dispose();
            if (flowFieldDirections.IsCreated) flowFieldDirections.Dispose();
            if (flowFieldDistance.IsCreated) flowFieldDistance.Dispose();
            if (flowFieldTargetPoints.IsCreated) flowFieldTargetPoints.Dispose();
            if (agentSpatialHash.IsCreated) agentSpatialHash.Dispose();
            if (obstaclePositions.IsCreated) obstaclePositions.Dispose();
            if (obstacleVelocities.IsCreated) obstacleVelocities.Dispose();
            if (obstacleRadii.IsCreated) obstacleRadii.Dispose();

            if (graph.IsCreated) graph.Dispose();
            if (triGrid.IsCreated) triGrid.Dispose();

            // bookkeeping gerenciado: só importa resetar quando isso é chamado como rede
            // de segurança em cima de um estado "sobrevivente" (ver Awake()) — cada
            // CustomNavMeshAgent/CustomNavMeshObstacle vai se RE-registrar sozinho no
            // OnEnable() da nova sessão de Play de qualquer forma (mesmo mecanismo que
            // faz Awake() deste manager rodar de novo), então começar com tudo vazio aqui
            // é exatamente o estado correto — equivalente a nunca ter tido nada registrado.
            agentComponents.Clear();
            obstacleComponents.Clear();
            pendingRepathSet.Clear();
            repathBatchBuffer.Clear();
            count = 0;
            obstacleCount = 0;
            graphReady = false;
        }

        // ==================== registro de agentes ====================

        public int RegisterAgent(CustomNavMeshAgent agent)
        {
            frameHandle.Complete(); // só é seguro mexer nos arrays sem job em voo

            if (count >= capacity)
            {
                Debug.LogError($"NavMeshJobManager: capacidade máxima de agentes ({capacity}) atingida; " +
                    "aumente 'Agent Capacity' no inspector do NavMeshJobManager.", agent);
                return -1;
            }

            int index = count++;
            // 'positions' é a verdade da simulação (rente à malha); se o Transform já foi
            // posicionado visualmente com o offset de Height, tira ele daqui pra começar consistente.
            positions[index] = (float3)agent.transform.position - new float3(0f, agent.Height, 0f);
            outPositions[index] = positions[index];
            velocities[index] = float3.zero;
            prevVelocities[index] = float3.zero;
            radii[index] = math.max(0f, agent.Radius); // defensivo — CustomNavMeshAgent.Radius já clampa, mas não custa garantir aqui também
            maxSpeeds[index] = agent.MaxSpeed;
            // defensivo (math.max) — CustomNavMeshAgent.WaypointReachDistance já clampa o
            // valor explícito, mas 'defaultWaypointReachDistance' é um campo serializado à
            // parte (Inspector do manager) que também precisa do mesmo piso.
            waypointReachDistances[index] = math.max(CustomNavMeshAgent.MinWaypointReachDistance,
                agent.WaypointReachDistance > 0f ? agent.WaypointReachDistance : defaultWaypointReachDistance);
            heights[index] = agent.Height;
            corridorCursor[index] = 0;
            corridorLength[index] = 0;
            pathStatus[index] = (byte)PathStatus.None;
            currentTriangle[index] = -1;
            movementFault[index] = (byte)MovementFaultType.None;
            movementFaultLogged[index] = false; // slot pode ter sido usado por outro agente antes (swap-remove reaproveita índice)
            paused[index] = false;
            ignoreAvoidance[index] = agent.IgnoreAvoidance; // AgentIndex do 'agent' ainda é -1 aqui, então o getter lê o campo serializado local
            neighborRadiusOverride[index] = -1f;
            timeHorizonOverride[index] = -1f;
            hasVelocityOverride[index] = false;
            flowFieldSlot[index] = -1;

            transformAccessArray.Add(agent.transform);
            agentComponents.Add(agent);

            return index;
        }

        public void UnregisterAgent(int index)
        {
            if (index < 0 || index >= count) return;

            frameHandle.Complete();

            int last = count - 1;

            // se o próprio agente removido estava com repath pendente, a pendência morre com ele
            pendingRepathSet.Remove(index);

            // idem pra flow field: o agente que está saindo do sistema libera sua referência
            // (isso é sobre o AGENTE removido, não sobre o slot 'index' do array — captura antes
            // de qualquer swap sobrescrever flowFieldSlot[index]).
            ReleaseFlowFieldRef(flowFieldSlot[index]);

            if (index != last)
            {
                positions[index] = positions[last];
                outPositions[index] = outPositions[last];
                velocities[index] = velocities[last];
                prevVelocities[index] = prevVelocities[last];
                radii[index] = radii[last];
                maxSpeeds[index] = maxSpeeds[last];
                waypointReachDistances[index] = waypointReachDistances[last];
                heights[index] = heights[last];
                corridorCursor[index] = corridorCursor[last];
                corridorLength[index] = corridorLength[last];
                pathStatus[index] = pathStatus[last];
                currentTriangle[index] = currentTriangle[last];
                movementFault[index] = movementFault[last];
                movementFaultLogged[index] = movementFaultLogged[last];
                paused[index] = paused[last];
                ignoreAvoidance[index] = ignoreAvoidance[last];
                neighborRadiusOverride[index] = neighborRadiusOverride[last];
                timeHorizonOverride[index] = timeHorizonOverride[last];
                velocityOverride[index] = velocityOverride[last];
                hasVelocityOverride[index] = hasVelocityOverride[last];
                flowFieldSlot[index] = flowFieldSlot[last]; // RefCount do slot não muda — o agente que ocupava 'last' continua usando o mesmo slot, só migrou de índice
                flowFieldPersonalTarget[index] = flowFieldPersonalTarget[last];

                int srcBase = last * NavMeshJobConstants.MaxCorridorPoints;
                int dstBase = index * NavMeshJobConstants.MaxCorridorPoints;
                for (int i = 0; i < NavMeshJobConstants.MaxCorridorPoints; i++)
                {
                    corridorFlat[dstBase + i] = corridorFlat[srcBase + i];
                    corridorIsLinkArrival[dstBase + i] = corridorIsLinkArrival[srcBase + i];
                }

                agentComponents[index] = agentComponents[last];
                agentComponents[index].AgentIndex = index;

                // o agente que ocupava 'last' carrega sua pendência de repath (se tinha) pro novo índice
                if (pendingRepathSet.Remove(last))
                    pendingRepathSet.Add(index);
            }

            agentComponents.RemoveAt(last);
            transformAccessArray.RemoveAtSwapBack(index);
            count--;
        }

        public void ClearCorridor(int index)
        {
            if (index < 0 || index >= count) return;
            frameHandle.Complete();
            corridorLength[index] = 0;
            corridorCursor[index] = 0;
            pathStatus[index] = (byte)PathStatus.None;

            ReleaseFlowFieldRef(flowFieldSlot[index]);
            flowFieldSlot[index] = -1;
        }

        // ==================== registro de obstáculos dinâmicos ====================

        public int RegisterObstacle(CustomNavMeshObstacle obstacle)
        {
            frameHandle.Complete(); // só é seguro mexer nos arrays sem job em voo

            if (obstacleCount >= obstacleCapacityInternal)
            {
                Debug.LogError($"NavMeshJobManager: capacidade máxima de obstáculos ({obstacleCapacityInternal}) " +
                    "atingida; aumente 'Obstacle Capacity' no inspector do NavMeshJobManager.", obstacle);
                return -1;
            }

            int index = obstacleCount++;
            obstaclePositions[index] = obstacle.transform.position;
            obstacleVelocities[index] = float3.zero; // sem histórico ainda — 1º frame sem estimativa de velocidade
            obstacleRadii[index] = math.max(0f, obstacle.Radius); // defensivo — CustomNavMeshObstacle.Radius já clampa, mas não custa garantir aqui também

            obstacleComponents.Add(obstacle);
            return index;
        }

        public void UnregisterObstacle(int index)
        {
            if (index < 0 || index >= obstacleCount) return;

            frameHandle.Complete();

            int last = obstacleCount - 1;
            if (index != last)
            {
                obstaclePositions[index] = obstaclePositions[last];
                obstacleVelocities[index] = obstacleVelocities[last];
                obstacleRadii[index] = obstacleRadii[last];

                obstacleComponents[index] = obstacleComponents[last];
                obstacleComponents[index].ObstacleIndex = index;
            }

            obstacleComponents.RemoveAt(last);
            obstacleCount--;
        }

        public float GetObstacleRadius(int index) => index >= 0 && index < obstacleCount ? obstacleRadii[index] : 0f;
        public void SetObstacleRadius(int index, float value) { if (index >= 0 && index < obstacleCount) obstacleRadii[index] = math.max(0f, value); }

        /// <summary>
        /// Lê Transform.position de cada obstáculo registrado (main thread — síncrono, mas o
        /// nº esperado de obstáculos dinâmicos é bem menor que o de agentes, então isso não
        /// é o mesmo tipo de gargalo que justificou o pipeline em Job pros agentes) e estima
        /// a velocidade por diferença de posição frame a frame — obstáculos não são
        /// simulados por este pacote (física/animação/outro script move o Transform deles
        /// livremente), então não há outra forma de saber a velocidade sem o jogo ter que
        /// chamar uma API explícita. Chamado no LateUpdate, ANTES de agendar os Jobs do
        /// frame — mesma garantia de "nenhum Job em voo" que o resto do pipeline.
        /// </summary>
        void RefreshObstacles()
        {
            if (obstacleCount == 0) return;

            float dt = Time.deltaTime;
            for (int i = 0; i < obstacleCount; i++)
            {
                var obstacle = obstacleComponents[i];
                if (obstacle == null) continue; // destruído sem passar por OnDisable (raro, ex.: Destroy direto na cena descarregando) — mantém a última posição conhecida

                float3 newPos = obstacle.transform.position;
                obstacleVelocities[i] = dt > 1e-5f ? (newPos - obstaclePositions[i]) / dt : float3.zero;
                obstaclePositions[i] = newPos;
            }
        }

        // ==================== pause / warp / avoidance override / tuning ao vivo ====================

        public void SetPaused(int index, bool value)
        {
            if (index >= 0 && index < count) paused[index] = value;
        }

        public bool GetPaused(int index) => index >= 0 && index < count && paused[index];

        /// <summary>
        /// Reposiciona o agente instantaneamente. Acha o triângulo mais próximo, zera
        /// velocidade, limpa corredor/flow field. Não mexe no destino do CustomNavMeshAgent
        /// (quem chama — CustomNavMeshAgent.Warp — decide se limpa isso também).
        /// </summary>
        /// <returns>false se o ponto está fora da área coberta pelo NavMesh, ou o grafo não está pronto.</returns>
        public bool Warp(int index, Vector3 worldPosition)
        {
            if (index < 0 || index >= count || !graphReady) return false;

            frameHandle.Complete(); // seguro mexer nos arrays sem job em voo

            int tri = NavMeshQueryUtil.FindNearestTriangle(worldPosition, triGrid, graph.Vertices, graph.Triangles, out float3 clamped);
            if (tri < 0) return false;

            positions[index] = clamped;
            outPositions[index] = clamped;
            velocities[index] = float3.zero;
            prevVelocities[index] = float3.zero;
            currentTriangle[index] = tri;

            corridorLength[index] = 0;
            corridorCursor[index] = 0;
            pathStatus[index] = (byte)PathStatus.None;

            ReleaseFlowFieldRef(flowFieldSlot[index]);
            flowFieldSlot[index] = -1;

            var t = agentComponents[index].transform;
            t.position = clamped + new float3(0f, heights[index], 0f);

            return true;
        }

        public void SetIgnoreAvoidance(int index, bool value)
        {
            if (index >= 0 && index < count) ignoreAvoidance[index] = value;
        }

        public bool GetIgnoreAvoidance(int index) => index >= 0 && index < count && ignoreAvoidance[index];

        /// <summary>Válido só até o próximo Update() resetar (ver ExpirePerFrameOverrides) — chame de novo todo frame pra manter.</summary>
        public void SetAvoidanceOverride(int index, float neighborQueryRadius, float timeHorizon)
        {
            if (index < 0 || index >= count) return;
            neighborRadiusOverride[index] = neighborQueryRadius;
            timeHorizonOverride[index] = timeHorizon;
        }

        public void ClearAvoidanceOverride(int index)
        {
            if (index < 0 || index >= count) return;
            neighborRadiusOverride[index] = -1f;
            timeHorizonOverride[index] = -1f;
        }

        /// <summary>
        /// Substitui a busca ativa de corredor/flow field por uma velocidade explícita — pra
        /// strafe, dodge, knockback, "andar pra frente" sem soltar o agente do sistema (continua
        /// clampado na malha, ainda sofre avoidance dos vizinhos, mas pula o clamp de MaxSpeed e
        /// a suavização de aceleração — é pra ser instantâneo). Válido só até o próximo LateUpdate
        /// resetar (mesmo contrato de SetAvoidanceOverride) — chame de novo todo frame enquanto
        /// quiser mantê-lo. Tem prioridade sobre Pause(): um agente pausado ainda se move se isso
        /// for chamado nele (é assim que um ataque corpo-a-corpo pausa o corredor e empurra o
        /// personagem pra frente no mesmo frame). Não mexe em CorridorCursor/flow field — quando
        /// parar de chamar, o agente retoma o corredor de onde estava.
        /// </summary>
        public void SetVelocityOverride(int index, Vector3 velocity)
        {
            if (index < 0 || index >= count) return;
            velocityOverride[index] = velocity;
            hasVelocityOverride[index] = true;
        }

        /// <summary>Normalmente desnecessário — expira sozinho se você simplesmente parar de chamar SetVelocityOverride.</summary>
        public void ClearVelocityOverride(int index)
        {
            if (index < 0 || index >= count) return;
            hasVelocityOverride[index] = false;
        }

        public float GetRadius(int index) => index >= 0 && index < count ? radii[index] : 0f;
        public void SetRadius(int index, float value) { if (index >= 0 && index < count) radii[index] = math.max(0f, value); }

        public float GetMaxSpeed(int index) => index >= 0 && index < count ? maxSpeeds[index] : 0f;
        public void SetMaxSpeed(int index, float value) { if (index >= 0 && index < count) maxSpeeds[index] = value; }

        public void SetHeight(int index, float value) { if (index >= 0 && index < count) heights[index] = value; }

        public void SetWaypointReachDistance(int index, float value) { if (index >= 0 && index < count) waypointReachDistances[index] = math.max(CustomNavMeshAgent.MinWaypointReachDistance, value); }

        /// <summary>True enquanto o pedido de repath individual de 'index' está na fila (ainda não processado por causa do budget de Max Path Requests Per Frame).</summary>
        public bool IsPathPending(int index) => pendingRepathSet.Contains(index);

        /// <summary>
        /// Distância restante estimada. Modo corredor: soma exata dos segmentos entre o waypoint
        /// atual e o fim. Modo flow field: aproximação (distância-ao-longo-do-campo até o
        /// triângulo de destino — não é a distância exata até o ponto de formação do agente).
        /// Infinito se o triângulo atual não tiver referência válida ou o alvo for inalcançável.
        /// </summary>
        public float GetRemainingDistance(int index)
        {
            if (index < 0 || index >= count) return 0f;

            int slot = flowFieldSlot[index];
            if (slot >= 0)
            {
                int tri = currentTriangle[index];
                if (tri < 0) return float.PositiveInfinity;

                float d = flowFieldDistance[slot * graph.TriangleCount + tri];
                return d >= float.MaxValue ? float.PositiveInfinity : d;
            }

            int len = corridorLength[index];
            if (len <= 0) return 0f;

            int cursor = corridorCursor[index];
            int baseIdx = index * NavMeshJobConstants.MaxCorridorPoints;
            float total = math.distance(positions[index], corridorFlat[baseIdx + cursor]);
            for (int p = cursor; p < len - 1; p++)
                total += math.distance(corridorFlat[baseIdx + p], corridorFlat[baseIdx + p + 1]);
            return total;
        }

        // ==================== flow field (grupo) ====================

        /// <summary>
        /// Move um grupo de agentes em direção ao destino usando um flow field compartilhado:
        /// UM cálculo (Dijkstra sobre todos os triângulos do NavMesh a partir do destino) serve
        /// pra qualquer nº de agentes em <paramref name="agents"/>, em vez de um A* por agente.
        /// Vale a pena quando muitos agentes convergem pro mesmo ponto (ex.: comando de grupo
        /// de RTS); pra um agente sozinho ou poucos agentes espalhados, SetDestination
        /// individual costuma ser mais barato (não paga o custo de Dijkstra sobre o grafo inteiro).
        ///
        /// <paramref name="keepFormation"/> (default true): em vez de todo mundo mirar o MESMO
        /// ponto de destino (o que aglomera o grupo num círculo apertado ao chegar), cada agente
        /// recebe seu próprio ponto de chegada — o destino deslocado pela posição relativa dele
        /// ao centróide do grupo NO MOMENTO desta chamada (preserva o formato/espaçamento atual
        /// do esquadrão, não impõe uma formação pré-definida). Esse deslocamento é rotacionado
        /// pra acompanhar a nova direção de deslocamento, estimada pela velocidade média atual
        /// do grupo (se o grupo estiver parado, não rotaciona — não há "frente" definida pra
        /// girar a partir dela). O flow field compartilhado guia todo mundo durante o trajeto
        /// longo (barato — um cálculo serve pro grupo inteiro); perto do alvo (Flow Field
        /// Arrive Distance) cada agente é automaticamente PROMOVIDO pro pipeline individual
        /// (SetDestination, A* + funnel) rumo ao seu ponto exato de formação — ver
        /// CheckFlowFieldArrivals. Isso combina o melhor dos dois mundos: o flow field resolve
        /// o trajeto em grupo sem pagar um A* por agente, e a reta final usa o corredor
        /// individual (já validado sem zigue-zague) que respeita a malha até o ponto exato,
        /// em vez de uma linha reta ingênua.
        ///
        /// Mutuamente exclusivo com SetDestination por agente: chamar SetDestination num agente
        /// que está num flow field o tira do grupo (volta a seguir corredor individual) no
        /// próximo frame — é exatamente esse mecanismo que o handoff automático usa por baixo
        /// dos panos; chamar MoveGroupWithFlowField de novo move o agente pra este (novo) campo.
        ///
        /// Síncrono: agenda e completa o ComputeFlowFieldJob na hora (não espera o próximo
        /// frame) — é uma chamada esporádica de comando de grupo, não um pedido por agente
        /// por frame, então o pequeno stall aqui é aceitável (mesmo padrão de RegisterAgent).
        /// </summary>
        /// <returns>false se o NavMesh não está pronto, o destino está fora da malha, ou não há slot livre no pool.</returns>
        public bool MoveGroupWithFlowField(CustomNavMeshAgent[] agents, Vector3 destination, uint areaMask = uint.MaxValue, bool keepFormation = true)
        {
            if (agents == null || agents.Length == 0) return false;

            if (!graphReady)
            {
                Debug.LogWarning("NavMeshJobManager: MoveGroupWithFlowField chamado sem NavMesh pronto.", this);
                return false;
            }

            frameHandle.Complete(); // seguro mexer nos arrays / rodar o job de cálculo do campo

            int targetTriangle = NavMeshQueryUtil.FindNearestTriangle(destination, triGrid, graph.Vertices, graph.Triangles, out float3 targetPoint);
            if (targetTriangle < 0)
            {
                Debug.LogWarning($"NavMeshJobManager: destino {destination} está fora da área coberta pelo NavMesh; MoveGroupWithFlowField ignorado.", this);
                return false;
            }

            if (!NavMeshQueryUtil.IsAreaAllowed(graph.TriangleArea[targetTriangle], areaMask))
            {
                Debug.LogWarning($"NavMeshJobManager: destino {destination} caiu numa área do NavMesh não permitida por areaMask; MoveGroupWithFlowField ignorado.", this);
                return false;
            }

            int slot = AcquireFlowFieldSlot();
            if (slot < 0)
            {
                Debug.LogError($"NavMeshJobManager: nenhum slot de flow field livre (Max Flow Fields = {maxFlowFields}, todos em uso). " +
                    "Aumente o valor no inspector ou espere grupos anteriores chegarem/trocarem de destino.", this);
                return false;
            }

            flowFieldMeta[slot].TargetTriangle = targetTriangle;
            flowFieldTargetPoints[slot] = targetPoint;

            new ComputeFlowFieldJob
            {
                Centers = graph.Centers,
                Neighbors = graph.Neighbors,
                AreaCost = graph.AreaCost,
                TriangleArea = graph.TriangleArea,
                Triangles = graph.Triangles,
                Vertices = graph.Vertices,
                TargetTriangle = targetTriangle,
                SlotOffset = slot * graph.TriangleCount,
                AreaMask = areaMask,
                DirectionsOut = flowFieldDirections,
                DistanceOut = flowFieldDistance,
            }.Schedule().Complete();

            // --- 1ª passada: centróide e velocidade média do grupo (só agentes válidos) ---
            float3 centroid = float3.zero;
            float3 avgVelocity = float3.zero;
            int validCount = 0;

            for (int i = 0; i < agents.Length; i++)
            {
                var agent = agents[i];
                if (agent == null || agent.AgentIndex < 0 || agent.AgentIndex >= count) continue;

                int idx = agent.AgentIndex;
                centroid += positions[idx];
                avgVelocity += velocities[idx];
                validCount++;
            }

            if (validCount == 0)
            {
                ReleaseFlowFieldRef(slot); // ninguém válido — não deixa o slot vazado
                return false;
            }

            centroid /= validCount;
            avgVelocity /= validCount;

            // ângulo pra girar o formato do grupo (capturado no espaço do mundo, orientação atual)
            // até apontar pra nova direção de deslocamento. Se o grupo está ~parado, não tem
            // "frente" definida pra girar a partir dela — fallback usa a própria direção de
            // deslocamento como referência, o que dá ângulo 0 (sem rotação).
            float rotationAngle = 0f;
            if (keepFormation)
            {
                float3 travelDir = targetPoint - centroid;
                float3 currentFacing = math.normalizesafe(avgVelocity, math.normalizesafe(travelDir));
                rotationAngle = SignedAngleXZ(currentFacing, math.normalizesafe(travelDir, currentFacing));
            }

            // --- 2ª passada: atribui slot + ponto de chegada individual ---
            int assigned = 0;
            for (int i = 0; i < agents.Length; i++)
            {
                var agent = agents[i];
                if (agent == null || agent.AgentIndex < 0 || agent.AgentIndex >= count) continue;

                int idx = agent.AgentIndex;
                pendingRepathSet.Remove(idx); // qualquer repath individual pendente perde a vez pro grupo

                int previousSlot = flowFieldSlot[idx];
                if (previousSlot != slot)
                {
                    ReleaseFlowFieldRef(previousSlot);
                    flowFieldSlot[idx] = slot;
                    flowFieldMeta[slot].RefCount++;
                }

                if (keepFormation)
                {
                    float3 offset = RotateY(positions[idx] - centroid, rotationAngle);
                    float3 desiredPoint = targetPoint + offset;
                    // reprojeta no NavMesh — o ponto deslocado pode cair fora da malha/atrás de
                    // um obstáculo perto do destino (ex.: formação larga perto de uma parede).
                    int desiredTri = NavMeshQueryUtil.FindNearestTriangle(desiredPoint, triGrid, graph.Vertices, graph.Triangles, out float3 clamped);
                    flowFieldPersonalTarget[idx] = desiredTri >= 0 ? clamped : targetPoint;
                }
                else
                {
                    flowFieldPersonalTarget[idx] = targetPoint;
                }

                corridorLength[idx] = 0;
                corridorCursor[idx] = 0;
                pathStatus[idx] = (byte)PathStatus.FlowField;
                assigned++;
            }

            if (assigned == 0)
            {
                ReleaseFlowFieldRef(slot); // não deveria acontecer (validCount>0 já garantiu isso), guarda mesmo assim
                return false;
            }

            return true;
        }

        /// <summary>Ângulo (radianos, sentido anti-horário visto de cima) de 'from' até 'to', projetados no plano XZ.</summary>
        static float SignedAngleXZ(float3 from, float3 to)
        {
            float2 f = from.xz;
            float2 t = to.xz;
            float dot = f.x * t.x + f.y * t.y;
            float det = f.x * t.y - f.y * t.x;
            return math.atan2(det, dot);
        }

        static float3 RotateY(float3 v, float angleRad)
        {
            float s = math.sin(angleRad);
            float c = math.cos(angleRad);
            return new float3(v.x * c - v.z * s, v.y, v.x * s + v.z * c);
        }

        int AcquireFlowFieldSlot()
        {
            for (int i = 0; i < flowFieldMeta.Length; i++)
            {
                if (!flowFieldMeta[i].InUse)
                {
                    flowFieldMeta[i].InUse = true;
                    flowFieldMeta[i].RefCount = 0;
                    return i;
                }
            }
            return -1;
        }

        void ReleaseFlowFieldRef(int slot)
        {
            if (slot < 0 || slot >= flowFieldMeta.Length) return;

            flowFieldMeta[slot].RefCount--;
            if (flowFieldMeta[slot].RefCount <= 0)
            {
                flowFieldMeta[slot].RefCount = 0;
                flowFieldMeta[slot].InUse = false; // buffers não são limpos — só serão sobrescritos no próximo uso do slot
            }
        }

        // ==================== consultas (main thread, pós-Complete) ====================

        public PathStatus GetStatus(int index) =>
            index >= 0 && index < count ? (PathStatus)pathStatus[index] : PathStatus.None;

        public float3 GetVelocity(int index) =>
            index >= 0 && index < count ? velocities[index] : float3.zero;

        /// <summary>Leitura O(1) — não faz nenhuma consulta nova, só reflete o triângulo já rastreado por frame (ver AvoidanceAndMoveJob.ClampToNavMesh). false só depois de um MovementFaultType.LostNavMesh persistente, ou antes do 1º frame do agente ter rodado.</summary>
        public bool GetIsOnNavMesh(int index) => index >= 0 && index < count && currentTriangle[index] >= 0;

        public bool HasReachedEnd(int index)
        {
            if (index < 0 || index >= count) return false;

            int slot = flowFieldSlot[index];
            if (slot >= 0)
            {
                int tri = currentTriangle[index];
                if (tri < 0) return false;

                float dist = flowFieldDistance[slot * graph.TriangleCount + tri];
                return dist <= flowFieldArriveDistance
                    && math.distance(positions[index], flowFieldPersonalTarget[index]) < waypointReachDistances[index];
            }

            int len = corridorLength[index];
            return len > 0 && corridorCursor[index] >= len - 1
                && math.distance(positions[index], corridorFlat[index * NavMeshJobConstants.MaxCorridorPoints + len - 1]) < waypointReachDistances[index];
        }

        // ==================== pipeline por frame ====================

        void Update()
        {
            // Job do frame anterior foi agendado E completado dentro do LateUpdate anterior
            // (mesma chamada) -- este Complete() aqui e so uma garantia idempotente. A partir
            // daqui ate o inicio do NOSSO LateUpdate(), nenhum Job esta em voo: e a janela
            // segura pra qualquer script de gameplay ler/escrever a API do CustomNavMeshAgent.
            frameHandle.Complete();
            if (frameRequests.IsCreated) frameRequests.Dispose();

            if (count > 0)
            {
                NativeArray<float3>.Copy(outPositions, positions, count);
                NativeArray<float3>.Copy(velocities, prevVelocities, count);
            }

            LogMovementFaults();
            ExpirePerFrameOverrides();

            CheckFlowFieldArrivals();
            CollectRepathRequests();
            // NAO agenda os Jobs aqui -- ver LateUpdate(). Agendar so depois que TODO Update()
            // da cena ja rodou garante que qualquer SetVelocityOverride/SetPaused/SetDestination
            // chamado por gameplay neste frame ja esta refletido nos arrays antes do Job ler.
        }

        void LateUpdate()
        {
            // So agora -- depois que TODOS os Update() da cena ja rodaram (inclusive scripts
            // de gameplay que leem/escrevem CustomNavMeshAgent) -- e seguro deixar o Job mexer
            // nos NativeArrays de novo. Agenda E completa aqui, na MESMA chamada: da pra
            // agendar mais cedo (em Update) e ganhar overlap com o resto do frame so se nada
            // mais tocar a API do agente entre o Schedule e o Complete -- como nao da pra
            // garantir isso pra scripts de terceiros (era exatamente a falha de seguranca
            // encontrada em revisao: NavMeshMovement.Update() lendo/escrevendo com o Job ja
            // em voo), esta e a opcao segura. O trabalho ainda roda em paralelo ENTRE os
            // agentes nos worker threads -- e isso que da o ganho real de perf sobre o
            // NavMeshAgent padrao, nao o overlap com outro codigo do frame (que era um
            // beneficio secundario, agora sacrificado em troca de correcao).
            RefreshObstacles(); // le Transform.position dos CustomNavMeshObstacle antes do Job usar
            ScheduleFrameJobs();
            frameHandle.Complete();
        }

        /// <summary>
        /// SetAvoidanceOverride/SetVelocityOverride são "válidos só neste frame" por design.
        /// Chamado no início do Update() (depois do Complete() do job agendado no LateUpdate
        /// anterior, que já leu/consumiu os valores setados no frame anterior) — resetar aqui,
        /// antes de qualquer script de gameplay rodar o Update() dele, dá zero atraso real: o
        /// gameplay pode setar de novo logo em seguida, no mesmo frame, e o LateUpdate() deste
        /// mesmo frame (que agenda o Job) já vai ler o valor fresco.
        /// </summary>
        void ExpirePerFrameOverrides()
        {
            for (int i = 0; i < count; i++)
            {
                neighborRadiusOverride[i] = -1f;
                timeHorizonOverride[i] = -1f;
                hasVelocityOverride[i] = false;
            }
        }

        /// <summary>
        /// Loga (uma vez por agente, pra não inundar o Console) qualquer MovementFault
        /// detectado por AvoidanceAndMoveJob nesse frame. É a única forma de saber que um
        /// travamento silencioso aconteceu — sem isso, nada em lugar nenhum acusa o problema
        /// (Burst não lança exceção por NaN, e o agente simplesmente para de se mover).
        /// </summary>
        void LogMovementFaults()
        {
            for (int i = 0; i < count; i++)
            {
                var fault = (MovementFaultType)movementFault[i];

                if (fault == MovementFaultType.None)
                {
                    movementFaultLogged[i] = false; // recuperado — a próxima ocorrência (se houver) loga de novo
                    continue;
                }

                if (movementFaultLogged[i]) continue; // já avisado desta ocorrência, não repete todo frame
                movementFaultLogged[i] = true;
                string agentName = agentComponents[i] != null ? agentComponents[i].name : $"agente #{i}";

                switch (fault)
                {
                    case MovementFaultType.InvalidVelocity:
                        Debug.LogWarning($"NavMeshJobManager: velocidade inválida (NaN/Infinity) detectada em '{agentName}' " +
                            $"e zerada antes de mover — provável triângulo degenerado no NavMesh baked ou combinação " +
                            "extrema de posição/velocidade no avoidance. Não deveria se repetir; se persistir, avise.",
                            agentComponents[i]);
                        break;
                    case MovementFaultType.LostNavMesh:
                        Debug.LogWarning($"NavMeshJobManager: '{agentName}' perdeu a referência de triângulo no NavMesh " +
                            "(nem o cache local nem a busca completa no grid acharam nada pra posição dele) — mantido " +
                            "parado na última posição válida. Se ele não se recuperar sozinho em alguns frames, " +
                            "ative Draw Status Gizmos pra ver onde ele está (esfera preta) e investigue a área.",
                            agentComponents[i]);
                        break;
                    case MovementFaultType.ExternalPositionAdopted:
                        Debug.Log($"NavMeshJobManager: Transform de '{agentName}' foi movido por fora da API " +
                            "(física/animação/cutscene mexendo direto no Transform) — posição externa adotada e " +
                            "reclampada no NavMesh. Normal se isso for esperado (ex.: knockback); se NÃO for, " +
                            "algum script está escrevendo transform.position diretamente em vez de usar " +
                            "SetVelocityOverride — ver README, 'Movendo o Transform por fora da API'.",
                            agentComponents[i]);
                        break;
                }
            }
        }

        /// <summary>
        /// Promove agentes em flow field pro pipeline individual (SetDestination) assim que
        /// entram no raio de Flow Field Arrive Distance. O flow field compartilhado cuida do
        /// trajeto longo em grupo (barato — um cálculo serve pra todo mundo); a reta final usa
        /// o mesmo A* + funnel do modo individual (já validado sem zigue-zague) até o ponto
        /// exato de chegada, respeitando a malha, em vez de uma linha reta ingênua até o ponto
        /// de formação. Só chama SetDestination — todo o resto (liberar o slot do campo,
        /// enfileirar o repath) já é tratado por CollectRepathRequests logo em seguida.
        /// </summary>
        void CheckFlowFieldArrivals()
        {
            for (int i = 0; i < count; i++)
            {
                int slot = flowFieldSlot[i];
                if (slot < 0) continue;

                int tri = currentTriangle[i];
                if (tri < 0) continue;

                float dist = flowFieldDistance[slot * graph.TriangleCount + tri];
                if (dist <= flowFieldArriveDistance)
                    agentComponents[i].SetDestination(flowFieldPersonalTarget[i]);
            }
        }

        void CollectRepathRequests()
        {
            for (int i = 0; i < count; i++)
            {
                var agent = agentComponents[i];
                if (agent != null && agent.HasDestination && agent.ConsumeDirtyFlag())
                {
                    // SetDestination foi chamado de novo: sai de qualquer flow field de grupo
                    // que estivesse usando (mutuamente exclusivo com corredor individual) e
                    // volta a seguir o pipeline normal de A* + funnel.
                    ReleaseFlowFieldRef(flowFieldSlot[i]);
                    flowFieldSlot[i] = -1;

                    pendingRepathSet.Add(i);
                }
            }
        }

        void ScheduleFrameJobs()
        {
            JobHandle pathHandle = default;
            int requestCount = 0;

            if (graphReady && pendingRepathSet.Count > 0)
            {
                repathBatchBuffer.Clear();
                foreach (int idx in pendingRepathSet)
                {
                    repathBatchBuffer.Add(idx);
                    if (repathBatchBuffer.Count >= maxPathRequestsPerFrame) break;
                }
                for (int i = 0; i < repathBatchBuffer.Count; i++)
                    pendingRepathSet.Remove(repathBatchBuffer[i]);

                requestCount = repathBatchBuffer.Count;
                frameRequests = new NativeList<PathRequest>(requestCount, Allocator.TempJob);
                for (int i = 0; i < requestCount; i++)
                {
                    int agentIdx = repathBatchBuffer[i];
                    var agent = agentComponents[agentIdx];
                    frameRequests.Add(new PathRequest
                    {
                        AgentIndex = agentIdx,
                        Start = positions[agentIdx],
                        End = agent.Destination,
                        AreaMask = agent.AreaMask,
                        Radius = radii[agentIdx],
                    });

                    // reseta o cursor NO MOMENTO em que o novo caminho é pedido (não dentro do
                    // Job): sem isso, um agente que estava no waypoint 5 do corredor ANTERIOR
                    // começaria o corredor NOVO também no índice 5, pulando curvas iniciais e
                    // mirando um waypoint que pode nem existir mais no caminho recalculado.
                    // Seguro fazer aqui (main thread, antes do Job ser agendado) — quando
                    // AvoidanceAndMoveJob rodar (depois de FindPathsBatchJob, por dependência),
                    // vai ler cursor=0 já pareado com o corredor recém-escrito.
                    corridorCursor[agentIdx] = 0;
                }

                pathHandle = new FindPathsBatchJob
                {
                    Requests = frameRequests.AsArray(),
                    Vertices = graph.Vertices,
                    Triangles = graph.Triangles,
                    Neighbors = graph.Neighbors,
                    Centers = graph.Centers,
                    AreaCost = graph.AreaCost,
                    TriangleArea = graph.TriangleArea,
                    Grid = triGrid,
                    LinkStart = graph.LinkStart,
                    LinkEnd = graph.LinkEnd,
                    LinkFromTriangle = graph.LinkFromTriangle,
                    LinkToTriangle = graph.LinkToTriangle,
                    LinkCost = graph.LinkCost,
                    LinkArea = graph.LinkArea,
                    LinkWidth = graph.LinkWidth,
                    CorridorOut = corridorFlat,
                    CorridorIsLinkArrivalOut = corridorIsLinkArrival,
                    CorridorLengthOut = corridorLength,
                    StatusOut = pathStatus,
                }.Schedule(requestCount, 4);
            }

            JobHandle moveHandle = pathHandle;

            if (count > 0 && graphReady)
            {
                agentSpatialHash.Clear();

                JobHandle hashHandle = new BuildAgentSpatialHashJob
                {
                    Positions = positions,
                    CellSize = neighborCellSize,
                    HashWriter = agentSpatialHash.AsParallelWriter(),
                }.Schedule(count, 32);

                // obstáculos dinâmicos (CustomNavMeshObstacle) no MESMO hashmap, índice
                // negativo — roda em paralelo com o hash de agentes (só ESCREVEM, chaves
                // tipicamente diferentes, sem corrida de dados) e entra na mesma combinação
                // de dependência antes do AvoidanceAndMoveJob.
                if (obstacleCount > 0)
                {
                    JobHandle obstacleHashHandle = new BuildObstacleSpatialHashJob
                    {
                        Positions = obstaclePositions,
                        CellSize = neighborCellSize,
                        HashWriter = agentSpatialHash.AsParallelWriter(),
                    }.Schedule(obstacleCount, 32);
                    hashHandle = JobHandle.CombineDependencies(hashHandle, obstacleHashHandle);
                }

                JobHandle deps = JobHandle.CombineDependencies(hashHandle, pathHandle);

                moveHandle = new AvoidanceAndMoveJob
                {
                    Positions = positions,
                    PrevVelocities = prevVelocities,
                    Radii = radii,
                    MaxSpeeds = maxSpeeds,
                    SpatialHash = agentSpatialHash,
                    ObstaclePositions = obstaclePositions,
                    ObstacleVelocities = obstacleVelocities,
                    ObstacleRadii = obstacleRadii,
                    CorridorFlat = corridorFlat,
                    CorridorIsLinkArrival = corridorIsLinkArrival,
                    CorridorLength = corridorLength,
                    WaypointReachDistances = waypointReachDistances,
                    FlowFieldSlot = flowFieldSlot,
                    FlowFieldDirections = flowFieldDirections,
                    FlowFieldDistances = flowFieldDistance,
                    FlowFieldPersonalTargets = flowFieldPersonalTarget,
                    FlowFieldTriangleCount = graph.TriangleCount,
                    FlowFieldArriveDistance = flowFieldArriveDistance,
                    Heights = heights,
                    CorridorCursor = corridorCursor,
                    TriGrid = triGrid,
                    NavVertices = graph.Vertices,
                    NavTriangles = graph.Triangles,
                    NavNeighbors = graph.Neighbors,
                    CurrentTriangle = currentTriangle,
                    OutPositions = outPositions,
                    OutVelocities = velocities,
                    DeltaTime = Time.deltaTime,
                    NeighborCellSize = neighborCellSize,
                    NeighborQueryRadius = neighborQueryRadius,
                    TimeHorizon = avoidanceTimeHorizon,
                    SteeringAccelerationFactor = steeringAccelerationFactor,
                    VerticalAvoidanceRange = verticalAvoidanceRange,
                    TriangleStickyMargin = triangleStickyMargin,
                    WallSafeBfsHops = wallSafeBfsHops,
                    ExternalMoveTolerance = externalMoveTolerance,
                    MovementFault = movementFault,
                    FlowFieldIgnoresAvoidance = flowFieldIgnoresAvoidance,
                    Paused = paused,
                    IgnoreAvoidance = ignoreAvoidance,
                    NeighborRadiusOverride = neighborRadiusOverride,
                    TimeHorizonOverride = timeHorizonOverride,
                    VelocityOverride = velocityOverride,
                    HasVelocityOverride = hasVelocityOverride,
                }.Schedule(transformAccessArray, deps);
            }

            frameHandle = moveHandle;
        }

        // ==================== debug ====================

        void OnDrawGizmos()
        {
            if (!Application.isPlaying) return;

            // desenhado mesmo sem grafo pronto — é justamente o caso que ele ajuda a flagrar
            // (todo mundo aparece cinza = nunca conseguiu nem tentar um path).
            if (drawStatusGizmos)
            {
                for (int i = 0; i < count; i++)
                {
                    Gizmos.color = StatusColor((PathStatus)pathStatus[i]);
                    Gizmos.DrawWireSphere(positions[i] + new float3(0f, 0.15f, 0f), radii[i] * 0.9f);

                    // marcador extra (esfera preta sólida) por cima de qualquer agente com
                    // MovementFault no frame atual — sobrepõe a cor de status normal, já que
                    // isso é mais grave/raro (ver LogMovementFaults / MovementFaultType).
                    if (movementFault[i] != (byte)MovementFaultType.None)
                    {
                        Gizmos.color = Color.black;
                        Gizmos.DrawSphere(positions[i] + new float3(0f, 0.15f, 0f), radii[i] * 0.4f);
                    }
                }
            }

            if (!graphReady) return;

            if (drawNavMeshGizmo)
            {
                Gizmos.color = new Color(0f, 1f, 1f, 0.25f);
                for (int t = 0; t < graph.TriangleCount; t++)
                {
                    int3 tri = graph.Triangles[t];
                    Vector3 a = graph.Vertices[tri.x];
                    Vector3 b = graph.Vertices[tri.y];
                    Vector3 c = graph.Vertices[tri.z];
                    Gizmos.DrawLine(a, b);
                    Gizmos.DrawLine(b, c);
                    Gizmos.DrawLine(c, a);
                }
            }

            if (drawCorridorGizmos)
            {
                Gizmos.color = Color.yellow;
                for (int i = 0; i < count; i++)
                {
                    int len = corridorLength[i];
                    int baseIdx = i * NavMeshJobConstants.MaxCorridorPoints;
                    for (int p = 0; p < len - 1; p++)
                        Gizmos.DrawLine(corridorFlat[baseIdx + p], corridorFlat[baseIdx + p + 1]);
                }
            }

            if (drawFlowFieldGizmo && flowFieldGizmoSlot >= 0 && flowFieldGizmoSlot < flowFieldMeta.Length
                && flowFieldMeta[flowFieldGizmoSlot].InUse)
            {
                Gizmos.color = Color.cyan;
                int offset = flowFieldGizmoSlot * graph.TriangleCount;
                for (int t = 0; t < graph.TriangleCount; t++)
                {
                    if (flowFieldDistance[offset + t] >= float.MaxValue) continue; // triângulo não alcançado pelo campo

                    float3 from = graph.Centers[t];
                    float3 dir = flowFieldDirections[offset + t];
                    if (math.lengthsq(dir) < 1e-6f) continue; // é o próprio triângulo de destino

                    Gizmos.DrawLine(from, from + dir * 0.75f);
                }

                // destino bruto do comando (branco) vs. ponto individual de cada agente do slot
                // (magenta) — mostra visualmente o quanto a formação está "abrindo" o alvo compartilhado.
                Gizmos.color = Color.white;
                Gizmos.DrawWireSphere(flowFieldTargetPoints[flowFieldGizmoSlot], 0.3f);

                Gizmos.color = Color.magenta;
                for (int i = 0; i < count; i++)
                {
                    if (flowFieldSlot[i] == flowFieldGizmoSlot)
                        Gizmos.DrawWireSphere(flowFieldPersonalTarget[i], 0.15f);
                }
            }
        }

        static Color StatusColor(PathStatus status)
        {
            switch (status)
            {
                case PathStatus.Success: return Color.green;
                case PathStatus.PartialCorridor: return Color.yellow;
                case PathStatus.NoPath: return Color.red;
                case PathStatus.Invalid: return Color.magenta;
                case PathStatus.FlowField: return Color.cyan;
                case PathStatus.BestEffort: return new Color(1f, 0.5f, 0f); // laranja — chegou perto, não no destino real
                default: return Color.gray; // None
            }
        }
    }
}
