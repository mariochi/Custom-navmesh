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

        public float Radius { get => radius; set => radius = value; }
        public float MaxSpeed { get => maxSpeed; set => maxSpeed = value; }
        public float WaypointReachDistance { get => waypointReachDistance; set => waypointReachDistance = value; }
        public uint AreaMask { get => areaMask; set => areaMask = value; }
        public float Height { get => height; set => height = value; }

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

        /// <summary>Para o agente e limpa o corredor atual (ele fica parado onde está).</summary>
        public void Stop()
        {
            hasDestination = false;
            destinationDirty = false;
            if (AgentIndex >= 0) NavMeshJobManager.Instance?.ClearCorridor(AgentIndex);
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
    }
}
