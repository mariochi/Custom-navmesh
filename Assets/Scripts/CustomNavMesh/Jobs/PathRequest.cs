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
    }
}
