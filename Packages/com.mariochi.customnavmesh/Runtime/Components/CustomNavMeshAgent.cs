using Unity.Mathematics;
using UnityEngine;

namespace CustomNavMesh
{
    /// <summary>
    /// Substituto do UnityEngine.AI.NavMeshAgent: não herda nada da API antiga, é só um
    /// "handle" que registra o Transform no NavMeshJobManager e expõe destino/velocidade/
    /// status. Todo o trabalho pesado (pathfinding, avoidance, movimento) acontece em Jobs
    /// dentro do manager — este componente não faz nada em Update.
    /// </summary>
    [DisallowMultipleComponent]
    public class CustomNavMeshAgent : MonoBehaviour
    {
        [SerializeField] float radius = 0.5f;
        [SerializeField] float maxSpeed = 3.5f;
        [SerializeField] float waypointReachDistance = 0.3f;
        [SerializeField] uint areaMask = uint.MaxValue;

        [Tooltip("Deslocamento vertical do Transform em relação ao ponto no NavMesh (equivalente ao " +
            "Base Offset do NavMeshAgent padrão). A simulação (pathfinding/avoidance/corredor) continua " +
            "rente à malha; só a posição final do Transform sobe por esse valor. Use se o pivot do seu " +
            "modelo estiver no centro/quadril em vez de nos pés — do contrário o personagem afunda " +
            "visualmente na malha até a altura do pivot.")]
        [SerializeField] float height = 0f;

        [Tooltip("Se marcado, este agente ignora avoidance por completo (equivalente a 'No Obstacle " +
            "Avoidance' do NavMeshAgent padrão) — segue reto pro alvo, atravessando outros agentes. " +
            "Útil pra unidades grandes/chefes que não devem ser desviados pela própria tropa.")]
        [SerializeField] bool ignoreAvoidance = false;

        // Radius/Height/MaxSpeed/WaypointReachDistance: o valor "fonte da verdade" depois do
        // registro passa a ser o NativeArray do manager (o setter escreve lá direto); o campo
        // serializado aqui só importa ANTES do registro (valor inicial) e se o componente for
        // desabilitado/reabilitado (novo registro lê o campo de novo).
        public float Radius
        {
            get => AgentIndex >= 0 && NavMeshJobManager.Instance != null ? NavMeshJobManager.Instance.GetRadius(AgentIndex) : radius;
            set
            {
                radius = value;
                if (AgentIndex >= 0) NavMeshJobManager.Instance?.SetRadius(AgentIndex, value);
            }
        }

        public float MaxSpeed
        {
            get => AgentIndex >= 0 && NavMeshJobManager.Instance != null ? NavMeshJobManager.Instance.GetMaxSpeed(AgentIndex) : maxSpeed;
            set
            {
                maxSpeed = value;
                if (AgentIndex >= 0) NavMeshJobManager.Instance?.SetMaxSpeed(AgentIndex, value);
            }
        }

        public float WaypointReachDistance
        {
            get => waypointReachDistance;
            set
            {
                waypointReachDistance = value;
                if (AgentIndex >= 0) NavMeshJobManager.Instance?.SetWaypointReachDistance(AgentIndex, value);
            }
        }

        public float Height
        {
            get => height;
            set
            {
                height = value;
                if (AgentIndex >= 0) NavMeshJobManager.Instance?.SetHeight(AgentIndex, value);
            }
        }

        public uint AreaMask { get => areaMask; set => areaMask = value; } // lido a cada repath (PathRequest), não cacheado — já é "vivo" por natureza

        /// <summary>
        /// Ignora avoidance por completo (equivalente a "No Obstacle Avoidance"). Ao contrário
        /// de <see cref="SetAvoidanceOverride"/>, isso é permanente até você trocar de novo (não
        /// expira sozinho a cada frame).
        /// </summary>
        public bool IgnoreAvoidance
        {
            get => AgentIndex >= 0 && NavMeshJobManager.Instance != null ? NavMeshJobManager.Instance.GetIgnoreAvoidance(AgentIndex) : ignoreAvoidance;
            set
            {
                ignoreAvoidance = value;
                if (AgentIndex >= 0) NavMeshJobManager.Instance?.SetIgnoreAvoidance(AgentIndex, value);
            }
        }

        internal int AgentIndex { get; set; } = -1;
        public PathStatus Status => AgentIndex >= 0 && NavMeshJobManager.Instance != null
            ? NavMeshJobManager.Instance.GetStatus(AgentIndex) : PathStatus.None;

        float3 destination;
        bool hasDestination;
        bool destinationDirty;

        void OnEnable()
        {
            if (NavMeshJobManager.Instance == null)
            {
                Debug.LogError("CustomNavMeshAgent: nenhum NavMeshJobManager encontrado na cena. Adicione um GameObject com esse componente antes dos agentes.", this);
                enabled = false;
                return;
            }

            AgentIndex = NavMeshJobManager.Instance.RegisterAgent(this);
            if (AgentIndex < 0) enabled = false;
        }

        void OnDisable()
        {
            if (AgentIndex >= 0 && NavMeshJobManager.Instance != null)
                NavMeshJobManager.Instance.UnregisterAgent(AgentIndex);
            AgentIndex = -1;
        }

        /// <summary>Pede um novo caminho até o destino informado (mundo). Processado em lote no próximo frame.</summary>
        public void SetDestination(Vector3 worldPosition)
        {
            destination = worldPosition;
            hasDestination = true;
            destinationDirty = true;
        }

        /// <summary>Para o agente e limpa o corredor atual (ele fica parado onde está, e esquece o destino — SetDestination de novo pra recomeçar).</summary>
        public void Stop()
        {
            hasDestination = false;
            destinationDirty = false;
            if (AgentIndex >= 0) NavMeshJobManager.Instance?.ClearCorridor(AgentIndex);
        }

        /// <summary>
        /// Trava a busca ativa do corredor/flow field (equivalente a NavMeshAgent.isStopped =
        /// true) SEM descartar o caminho — Resume() retoma exatamente de onde parou. Diferente
        /// de Stop(): o destino/corredor continuam intactos. O agente ainda participa do
        /// avoidance (outros o veem como obstáculo; ele reage se empurrado).
        /// </summary>
        public void Pause()
        {
            if (AgentIndex >= 0) NavMeshJobManager.Instance?.SetPaused(AgentIndex, true);
        }

        /// <summary>Retoma a busca do corredor/flow field depois de Pause().</summary>
        public void Resume()
        {
            if (AgentIndex >= 0) NavMeshJobManager.Instance?.SetPaused(AgentIndex, false);
        }

        public bool IsPaused => AgentIndex >= 0 && NavMeshJobManager.Instance != null && NavMeshJobManager.Instance.GetPaused(AgentIndex);

        /// <summary>Leitura O(1) — não faz nenhuma consulta nova por trás, só reflete o triângulo já rastreado por frame. Útil pra checagens todo-frame (ex.: EnsureOnNavMesh) sem pagar o custo de um NavMesh.SamplePosition síncrono.</summary>
        public bool IsOnNavMesh => AgentIndex >= 0 && NavMeshJobManager.Instance != null && NavMeshJobManager.Instance.GetIsOnNavMesh(AgentIndex);

        /// <summary>
        /// Reposiciona o agente instantaneamente (sem interpolar), tipo respawn ou pouso pós-
        /// movimento forçado. Descarta o corredor/destino atual — chame SetDestination de novo
        /// se quiser que ele continue andando pra algum lugar depois do warp.
        /// </summary>
        /// <returns>false se o ponto está fora da área coberta pelo NavMesh (nada muda nesse caso).</returns>
        public bool Warp(Vector3 worldPosition)
        {
            if (AgentIndex < 0 || NavMeshJobManager.Instance == null) return false;

            bool ok = NavMeshJobManager.Instance.Warp(AgentIndex, worldPosition);
            if (ok)
            {
                hasDestination = false;
                destinationDirty = false;
            }
            return ok;
        }

        /// <summary>
        /// Override de avoidance válido só pelo frame atual — chame de novo todo frame enquanto
        /// quiser mantê-lo ativo (ex.: enquanto o agente estiver dentro de uma zona de gargalo);
        /// se parar de chamar, volta ao valor global do NavMeshJobManager sozinho no frame seguinte.
        /// </summary>
        public void SetAvoidanceOverride(float neighborQueryRadius, float timeHorizon)
        {
            if (AgentIndex >= 0) NavMeshJobManager.Instance?.SetAvoidanceOverride(AgentIndex, neighborQueryRadius, timeHorizon);
        }

        /// <summary>Remove o override antes do fim do frame (normalmente desnecessário — ele já expira sozinho se você simplesmente parar de chamar SetAvoidanceOverride).</summary>
        public void ClearAvoidanceOverride()
        {
            if (AgentIndex >= 0) NavMeshJobManager.Instance?.ClearAvoidanceOverride(AgentIndex);
        }

        /// <summary>
        /// Substitui a busca ativa de corredor/flow field por uma velocidade explícita — pra
        /// strafe, dodge, knockback e "andar pra frente" sem soltar o agente do sistema (continua
        /// clampado na malha, ainda sofre avoidance dos vizinhos). Válido só neste frame, mesmo
        /// contrato de SetAvoidanceOverride — chame de novo todo frame enquanto quiser mantê-lo.
        /// Tem prioridade sobre Pause(): um agente pausado ainda se move se isto for chamado
        /// (é assim que um ataque corpo-a-corpo consegue pausar o corredor e empurrar o
        /// personagem pra frente no mesmo frame). Pula o clamp de MaxSpeed e a suavização de
        /// aceleração (SteeringAccelerationFactor) — é pra ser instantâneo, sem rampa; ainda
        /// assim não atravessa parede (ClampToNavMesh continua rodando incondicionalmente).
        /// </summary>
        public void SetVelocityOverride(Vector3 velocity)
        {
            if (AgentIndex >= 0) NavMeshJobManager.Instance?.SetVelocityOverride(AgentIndex, velocity);
        }

        /// <summary>Normalmente desnecessário — expira sozinho se você simplesmente parar de chamar SetVelocityOverride.</summary>
        public void ClearVelocityOverride()
        {
            if (AgentIndex >= 0) NavMeshJobManager.Instance?.ClearVelocityOverride(AgentIndex);
        }

        internal bool HasDestination => hasDestination;
        internal float3 Destination => destination;

        internal bool ConsumeDirtyFlag()
        {
            if (!destinationDirty) return false;
            destinationDirty = false;
            return true;
        }

        public Vector3 Velocity => AgentIndex >= 0 && NavMeshJobManager.Instance != null
            ? (Vector3)NavMeshJobManager.Instance.GetVelocity(AgentIndex) : Vector3.zero;

        public bool HasArrived => AgentIndex >= 0 && NavMeshJobManager.Instance != null
            && NavMeshJobManager.Instance.HasReachedEnd(AgentIndex);

        /// <summary>
        /// True se o agente está sendo movido por um flow field de grupo
        /// (NavMeshJobManager.MoveGroupWithFlowField) em vez de seguir um corredor
        /// individual. Chamar SetDestination tira o agente do flow field no próximo frame.
        /// </summary>
        public bool IsUsingFlowField => Status == PathStatus.FlowField;

        /// <summary>
        /// Distância restante ao longo do corredor/flow field (soma dos segmentos entre o
        /// waypoint atual e o fim) — não é linha reta até o destino. No modo flow field, é uma
        /// aproximação (distância-ao-longo-do-campo até o triângulo de destino, não o caminho
        /// exato até o ponto de formação). Infinito se o destino for inalcançável.
        /// </summary>
        public float RemainingDistance => AgentIndex >= 0 && NavMeshJobManager.Instance != null
            ? NavMeshJobManager.Instance.GetRemainingDistance(AgentIndex) : 0f;

        /// <summary>True enquanto um pedido de caminho individual (SetDestination) está na fila esperando processamento (respeitando Max Path Requests Per Frame) — ainda não vale nada em modo flow field.</summary>
        public bool IsPathPending => AgentIndex >= 0 && NavMeshJobManager.Instance != null
            && NavMeshJobManager.Instance.IsPathPending(AgentIndex);
    }
}
