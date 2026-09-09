using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>Um pedido de caminho pra um agente, consumido pelo FindPathsBatchJob.</summary>
    public struct PathRequest
    {
        /// <summary>
        /// Índice do agente no NavMeshJobManager. É por ESSE índice (não pelo índice do
        /// request dentro do batch) que os resultados são escritos — cada AgentIndex deve
        /// aparecer no máximo uma vez por batch (ver comentário em FindPathsBatchJob).
        /// </summary>
        public int AgentIndex;

        public float3 Start;
        public float3 End;

        /// <summary>Bitmask de áreas permitidas (bit N = área N). uint.MaxValue = todas.</summary>
        public uint AreaMask;

        /// <summary>
        /// Raio do agente — usado por Funnel.BuildCorridor pra encolher cada portal
        /// interno (aresta compartilhada entre dois triângulos do corredor) pra dentro por
        /// até esse valor antes de rodar o funil, mantendo o corredor gerado a pelo menos
        /// Radius de distância de qualquer aresta/vértice interno no caminho (ver
        /// comentário em Funnel.BuildCorridor). 0 = sem margem nenhuma (comportamento de
        /// antes desse campo existir — corredor colado exatamente na geometria).
        /// </summary>
        public float Radius;
    }
}
