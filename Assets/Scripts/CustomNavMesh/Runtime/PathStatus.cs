namespace CustomNavMesh
{
    public enum PathStatus : byte
    {
        /// <summary>Nenhum caminho foi calculado ainda pra esse agente.</summary>
        None = 0,

        /// <summary>Caminho completo encontrado, corredor cabe no buffer.</summary>
        Success = 1,

        /// <summary>Caminho encontrado, mas era mais longo que MaxCorridorPoints e foi truncado.</summary>
        PartialCorridor = 2,

        /// <summary>Início e fim são válidos, mas não existe caminho conectando os dois (ilhas de NavMesh diferentes, etc).</summary>
        NoPath = 3,

        /// <summary>Início ou fim não encontraram nenhum triângulo próximo (fora do alcance do NavMesh).</summary>
        Invalid = 4,

        /// <summary>Agente sendo movido por um flow field de grupo (MoveGroupWithFlowField) em vez de corredor individual.</summary>
        FlowField = 5,
    }
}
