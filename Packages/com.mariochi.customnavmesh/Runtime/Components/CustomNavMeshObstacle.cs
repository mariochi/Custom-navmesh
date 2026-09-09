using Unity.Mathematics;
using UnityEngine;

namespace CustomNavMesh
{
    /// <summary>
    /// Obstáculo DINÂMICO pra avoidance: um objeto que se move por conta própria (física de
    /// Rigidbody, animação, um script de terceiros — qualquer coisa que não seja um
    /// CustomNavMeshAgent deste pacote) e que os agentes devem desviar, sem que ele próprio
    /// seja movido pelo nosso pipeline. Exemplos: uma pedra rolando, uma porta se fechando
    /// devagar, uma plataforma móvel, o jogador (se não for um agente).
    ///
    /// Diferente de um NavMeshObstacle nativo (que é sobre TOPOLOGIA — carving, requer
    /// RebuildGraph pra ter efeito, serve pra coisas praticamente estáticas), isto é sobre
    /// AVOIDANCE LOCAL em tempo real: o obstáculo vira uma restrição ORCA extra no espaço de
    /// velocidades de cada agente próximo, recalculada todo frame a partir da posição atual
    /// (e velocidade estimada por diferença de posição) — sem precisar reconstruir o grafo.
    ///
    /// A responsabilidade de evitar colisão aqui é 100% do AGENTE (não-recíproca): o
    /// obstáculo não sabe nada sobre ORCA e não desvia de ninguém — ao contrário de dois
    /// CustomNavMeshAgent, que dividem a responsabilidade 50/50 (ver AvoidanceAndMoveJob).
    ///
    /// Este componente só REGISTRA posição/raio pro NavMeshJobManager ler a cada frame
    /// (Update(), main thread — a leitura de Transform.position de N obstáculos é OK ser
    /// síncrona porque o nº esperado de obstáculos é bem menor que o de agentes). Ele não
    /// tem AgentIndex, não participa de pathfinding, não é afetado por avoidance nenhum.
    /// </summary>
    [DisallowMultipleComponent]
    public class CustomNavMeshObstacle : MonoBehaviour
    {
        [Tooltip("Raio (metros) do disco usado como obstáculo pra avoidance dos agentes próximos.")]
        [SerializeField] float radius = 0.5f;

        [Tooltip("Manager específico que este obstáculo deve usar — deixe vazio (default) pra usar " +
            "o NavMeshJobManager.Instance padrão. Ver campo equivalente em CustomNavMeshAgent pro " +
            "motivo de existir (múltiplas instâncias de NavMeshJobManager coexistindo, ver README, " +
            "'Múltiplas instâncias / multi-cena') — um obstáculo só afeta avoidance dos agentes " +
            "registrados NESSE MESMO manager, então em setups multi-manager ele precisa apontar pro " +
            "manager certo explicitamente, senão nenhum agente da área pretendida o vê.")]
        [SerializeField] NavMeshJobManager manager;

        /// <summary>Manager que este obstáculo usa: o 'Manager' explícito acima se setado, senão NavMeshJobManager.Instance (o padrão da cena/processo).</summary>
        public NavMeshJobManager Manager => manager != null ? manager : NavMeshJobManager.Instance;

        public float Radius
        {
            get => ObstacleIndex >= 0 && Manager != null ? Manager.GetObstacleRadius(ObstacleIndex) : radius;
            set
            {
                // clampado aqui (não só no manager) — mesmo motivo do CustomNavMeshAgent.Radius:
                // um raio negativo inverteria parte da geometria da reta ORCA em vez de ser
                // rejeitado.
                radius = math.max(0f, value);
                if (ObstacleIndex >= 0) Manager?.SetObstacleRadius(ObstacleIndex, radius);
            }
        }

        /// <summary>Índice interno no NavMeshJobManager, ou -1 se não registrado. Não confundir com CustomNavMeshAgent.AgentIndex — são pools separados.</summary>
        public int ObstacleIndex { get; internal set; } = -1;

        // Clampa o campo serializado no Inspector — ver comentário equivalente em
        // CustomNavMeshAgent.OnValidate().
        void OnValidate() => radius = math.max(0f, radius);

        void OnEnable()
        {
            if (Manager == null)
            {
                Debug.LogError("CustomNavMeshObstacle: nenhum NavMeshJobManager encontrado (nem um 'Manager' " +
                    "explícito, nem um Instance padrão na cena). Adicione um GameObject com esse componente " +
                    "antes dos obstáculos, ou aponte o campo 'Manager' deste obstáculo pra um específico.", this);
                enabled = false;
                return;
            }

            ObstacleIndex = Manager.RegisterObstacle(this);
            if (ObstacleIndex < 0) enabled = false;
        }

        void OnDisable()
        {
            if (ObstacleIndex >= 0 && Manager != null)
                Manager.UnregisterObstacle(ObstacleIndex);
            ObstacleIndex = -1;
        }
    }
}
