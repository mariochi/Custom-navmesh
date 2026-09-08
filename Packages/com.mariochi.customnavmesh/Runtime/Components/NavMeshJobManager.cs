using System.Collections;
using System.Collections.Generic;
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
    /// Pipeline: Schedule em Update() (roda em worker threads durante o resto do Update de
    /// outros scripts), Complete() em LateUpdate() (ponto de sincronização, antes de
    /// física/câmera). Isso é o que dá o ganho real de multithreading sobre o
    /// NavMeshAgent padrão do Unity, que resolve tudo de forma síncrona na main thread.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class NavMeshJobManager : MonoBehaviour
    {
        [Header("Capacidade")]
        [Tooltip("Número máximo de agentes registrados simultaneamente. Os NativeArrays são alocados com esse tamanho fixo no Awake.")]
        [SerializeField] int agentCapacity = 256;
        [Tooltip("Quantos pedidos de repath (novo destino) são processados por frame, no máximo. Limita picos de custo quando muitos agentes pedem caminho no mesmo frame.")]
        [SerializeField] int maxPathRequestsPerFrame = 32;

        [Header("Grid do NavMesh (broad-phase de triângulo)")]
        [SerializeField] float triangleGridCellSize = 4f;
        [Tooltip("Distância pra considerar dois vértices da triangulação como o mesmo ponto (solda) " +
            "ao montar o grafo de adjacência. Necessário porque NavMesh.CalculateTriangulation() não " +
            "garante índice de vértice compartilhado na costura entre tiles do NavMesh (comum em mapas " +
            "grandes) — sem soldar, cada tile vira uma ilha isolada e o A* nunca acha caminho entre eles.")]
        [SerializeField] float vertexWeldEpsilon = NavMeshGraphBuilder.DefaultWeldEpsilon;
        [Tooltip("Se marcado, o manager reconstrói o grafo sozinho (RebuildGraph + RepathAllAgents) " +
            "sempre que o NavMesh mudar em runtime (NavMeshObstacle fazendo carving, NavMeshLink " +
            "ativado/desativado, etc.) — inscrito em NavMesh.onPreUpdate. Várias mudanças próximas no " +
            "tempo viram uma única reconstrução (ver Auto Rebuild Debounce). Desligue se preferir " +
            "controlar manualmente quando chamar RebuildGraph().")]
        [SerializeField] bool autoRebuildOnNavMeshChange = true;
        [Tooltip("Janela de silêncio (segundos) depois da última mudança detectada no NavMesh antes " +
            "de reconstruir o grafo — evita reconstruir uma vez por frame quando várias mudanças " +
            "acontecem em sequência rápida (ex.: vários NavMeshObstacle entrando em cena juntos).")]
        [SerializeField] float autoRebuildDebounce = 0.25f;

        [Header("Avoidance")]
        [SerializeField] float neighborCellSize = 2f;
        [SerializeField] float neighborQueryRadius = 4f;
        [SerializeField] float avoidanceTimeHorizon = 2f;
        [SerializeField] float defaultWaypointReachDistance = 0.3f;
        [Tooltip("Limite de variação de velocidade, como múltiplo de MaxSpeed por segundo (ex.: 8 = " +
            "sai do repouso até a velocidade máxima em ~1/8s). Suaviza mudanças bruscas de direção " +
            "(cruzar de triângulo no flow field, convergência de grupo perto do destino) em vez de " +
            "deixar a velocidade saltar direto pro valor desejado todo frame. 0 desliga a suavização.")]
        [SerializeField] float steeringAccelerationFactor = 10f;
        [Tooltip("0-1: quanto o empurrão de sobreposição é amortecido quando os dois agentes já " +
            "andam na mesma direção (marchando juntos — comum num grupo grande no mesmo flow " +
            "field). 1 = sem amortecimento; valores menores (ex.: 0.3) acalmam oscilação em " +
            "multidão densa/gargalo sem enfraquecer a resposta a colisões de frente/cruzadas.")]
        [SerializeField, Range(0f, 1f)] float crowdPushDamping = 0.3f;

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
            "ciano = FlowField (seguindo campo de grupo).")]
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
        NativeArray<byte> movementFault; // MovementFaultType do frame atual, por agente — diagnóstico (ver AvoidanceAndMoveJob)
        NativeArray<bool> movementFaultLogged; // já logamos esse agente pelo menos uma vez (evita spam no Console)
        NativeArray<bool> paused; // Pause()/Resume() — trava a busca ativa sem descartar corredor/flow field
        NativeArray<bool> ignoreAvoidance; // por agente, permanente até trocar de novo
        NativeArray<float> neighborRadiusOverride; // -1 = usa o global; resetado todo frame em LateUpdate (válido só 1 frame)
        NativeArray<float> timeHorizonOverride; // -1 = usa o global; idem

        // --- flow field: por agente (capacidade fixa) + pool de campos achatado (depende de TriangleCount) ---
        NativeArray<int> flowFieldSlot; // -1 = agente no modo corredor
        NativeArray<float3> flowFieldDirections; // flat: slot * graph.TriangleCount + triângulo
        NativeArray<float> flowFieldDistance;
        NativeArray<float3> flowFieldTargetPoints; // por slot (tamanho maxFlowFields) — ponto "bruto" do comando, sem formação
        NativeArray<float3> flowFieldPersonalTarget; // por agente — ponto que ele mira de fato perto do alvo (com offset de formação, se houver)
        FlowFieldMeta[] flowFieldMeta; // bookkeeping gerenciado, pequeno, não precisa ser NativeArray

        NativeParallelMultiHashMap<int, int> agentSpatialHash;

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

        void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Debug.LogWarning("NavMeshJobManager: já existe uma instância nessa cena; destruindo a duplicada.", this);
                Destroy(this);
                return;
            }
            Instance = this;

            capacity = math.max(1, agentCapacity);
            AllocatePersistent();
        }

        Coroutine autoRebuildCoroutine;
        float autoRebuildDeadline;

        void OnEnable()
        {
            // NavMesh.onPreUpdate dispara todo frame em que o Unity processa atualização de
            // NavMesh — inclusive carving de NavMeshObstacle e NavMeshLink ligando/desligando.
            // É a forma pública de saber "algo pode ter mudado" sem o jogo precisar chamar
            // RebuildGraph() manualmente toda vez que mexe num obstáculo/portão.
            NavMesh.onPreUpdate += HandleNavMeshPreUpdate;
        }

        void OnDisable()
        {
            NavMesh.onPreUpdate -= HandleNavMeshPreUpdate;
        }

        void HandleNavMeshPreUpdate()
        {
            if (!autoRebuildOnNavMeshChange) return;

            // debounce: cada chamada empurra o prazo pra frente; a corrotina só executa o
            // rebuild de fato depois de 'autoRebuildDebounce' segundos SEM nenhuma chamada nova
            // — várias mudanças em sequência rápida (vários obstáculos entrando juntos) viram
            // uma única reconstrução em vez de uma por frame.
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

        void Start()
        {
            // NÃO construir o grafo no Awake: com [DefaultExecutionOrder(-100)] o Awake deste
            // manager roda antes do OnEnable de outros scripts da cena — incluindo o
            // NavMeshSurface, que só registra os dados baked em NavMesh.AddNavMeshData()
            // no próprio OnEnable. O Unity garante que TODO Awake/OnEnable da cena termina
            // antes de QUALQUER Start(), independente de execution order — então esperar até
            // aqui garante que NavMesh.CalculateTriangulation() já vai enxergar a malha.
            if (Instance == this)
                RebuildGraph();
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
            movementFault = new NativeArray<byte>(capacity, Allocator.Persistent);
            movementFaultLogged = new NativeArray<bool>(capacity, Allocator.Persistent);
            paused = new NativeArray<bool>(capacity, Allocator.Persistent);
            ignoreAvoidance = new NativeArray<bool>(capacity, Allocator.Persistent);
            neighborRadiusOverride = new NativeArray<float>(capacity, Allocator.Persistent);
            timeHorizonOverride = new NativeArray<float>(capacity, Allocator.Persistent);
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

            agentSpatialHash = new NativeParallelMultiHashMap<int, int>(math.max(64, capacity), Allocator.Persistent);
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
            triGrid = NavMeshSpatialGrid.Build(graph, Allocator.Persistent, triangleGridCellSize);
            graphReady = graph.TriangleCount > 0;

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
                Debug.Log($"NavMeshJobManager: grafo reconstruído — {graph.TriangleCount} triângulos, " +
                    $"grid {triGrid.CellCount.x}x{triGrid.CellCount.y} células de {triangleGridCellSize}u " +
                    $"cobrindo ~{coveredX:F0}x{coveredZ:F0}u. Se path/registro de agente parecer lento, " +
                    $"ajuste 'Triangle Grid Cell Size' (ver README, seção 'mapas grandes').", this);
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
            if (movementFault.IsCreated) movementFault.Dispose();
            if (movementFaultLogged.IsCreated) movementFaultLogged.Dispose();
            if (paused.IsCreated) paused.Dispose();
            if (ignoreAvoidance.IsCreated) ignoreAvoidance.Dispose();
            if (neighborRadiusOverride.IsCreated) neighborRadiusOverride.Dispose();
            if (timeHorizonOverride.IsCreated) timeHorizonOverride.Dispose();
            if (flowFieldSlot.IsCreated) flowFieldSlot.Dispose();
            if (flowFieldPersonalTarget.IsCreated) flowFieldPersonalTarget.Dispose();
            if (flowFieldDirections.IsCreated) flowFieldDirections.Dispose();
            if (flowFieldDistance.IsCreated) flowFieldDistance.Dispose();
            if (flowFieldTargetPoints.IsCreated) flowFieldTargetPoints.Dispose();
            if (agentSpatialHash.IsCreated) agentSpatialHash.Dispose();

            if (graph.IsCreated) graph.Dispose();
            if (triGrid.IsCreated) triGrid.Dispose();

            if (Instance == this) Instance = null;
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
            radii[index] = agent.Radius;
            maxSpeeds[index] = agent.MaxSpeed;
            waypointReachDistances[index] = agent.WaypointReachDistance > 0f ? agent.WaypointReachDistance : defaultWaypointReachDistance;
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
                flowFieldSlot[index] = flowFieldSlot[last]; // RefCount do slot não muda — o agente que ocupava 'last' continua usando o mesmo slot, só migrou de índice
                flowFieldPersonalTarget[index] = flowFieldPersonalTarget[last];

                int srcBase = last * NavMeshJobConstants.MaxCorridorPoints;
                int dstBase = index * NavMeshJobConstants.MaxCorridorPoints;
                for (int i = 0; i < NavMeshJobConstants.MaxCorridorPoints; i++)
                    corridorFlat[dstBase + i] = corridorFlat[srcBase + i];

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

        /// <summary>Válido só até o próximo LateUpdate resetar (ver comentário lá) — chame de novo todo frame pra manter.</summary>
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

        public float GetRadius(int index) => index >= 0 && index < count ? radii[index] : 0f;
        public void SetRadius(int index, float value) { if (index >= 0 && index < count) radii[index] = value; }

        public float GetMaxSpeed(int index) => index >= 0 && index < count ? maxSpeeds[index] : 0f;
        public void SetMaxSpeed(int index, float value) { if (index >= 0 && index < count) maxSpeeds[index] = value; }

        public void SetHeight(int index, float value) { if (index >= 0 && index < count) heights[index] = value; }

        public void SetWaypointReachDistance(int index, float value) { if (index >= 0 && index < count) waypointReachDistances[index] = value; }

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
            // 'frameHandle' do frame anterior já deveria ter sido completado no LateUpdate
            // anterior; Complete() aqui é só uma garantia (idempotente/rápida se já concluído).
            frameHandle.Complete();
            if (frameRequests.IsCreated) frameRequests.Dispose();

            CheckFlowFieldArrivals();
            CollectRepathRequests();
            ScheduleFrameJobs();
        }

        void LateUpdate()
        {
            // ponto de sincronização: garante que posição/velocidade estejam prontas antes
            // de física, câmera, animação etc. lerem o Transform dos agentes nesse frame.
            frameHandle.Complete();

            if (count > 0)
            {
                NativeArray<float3>.Copy(outPositions, positions, count);
                NativeArray<float3>.Copy(velocities, prevVelocities, count);
            }

            LogMovementFaults();
            ExpireAvoidanceOverrides();
        }

        /// <summary>
        /// SetAvoidanceOverride é "válido só neste frame" por design: o job deste frame já
        /// consumiu os valores atuais (Update -> ScheduleFrameJobs, antes de qualquer script de
        /// gameplay de execution order default rodar), então é seguro resetar pra -1 aqui — quem
        /// quiser manter o override simplesmente chama SetAvoidanceOverride nele nesse mesmo
        /// frame, definindo o valor que vale pro PRÓXIMO frame (mesmo atraso de 1 frame que já
        /// existe em Positions/PrevVelocities).
        /// </summary>
        void ExpireAvoidanceOverrides()
        {
            for (int i = 0; i < count; i++)
            {
                neighborRadiusOverride[i] = -1f;
                timeHorizonOverride[i] = -1f;
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
                    });
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
                    CorridorOut = corridorFlat,
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

                JobHandle deps = JobHandle.CombineDependencies(hashHandle, pathHandle);

                moveHandle = new AvoidanceAndMoveJob
                {
                    Positions = positions,
                    PrevVelocities = prevVelocities,
                    Radii = radii,
                    MaxSpeeds = maxSpeeds,
                    SpatialHash = agentSpatialHash,
                    CorridorFlat = corridorFlat,
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
                    CrowdPushDamping = crowdPushDamping,
                    MovementFault = movementFault,
                    FlowFieldIgnoresAvoidance = flowFieldIgnoresAvoidance,
                    Paused = paused,
                    IgnoreAvoidance = ignoreAvoidance,
                    NeighborRadiusOverride = neighborRadiusOverride,
                    TimeHorizonOverride = timeHorizonOverride,
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
                default: return Color.gray; // None
            }
        }
    }
}
