namespace CustomNavMesh
{
    /// <summary>
    /// Constantes compartilhadas entre os Jobs e o manager. Ficam num só lugar pra
    /// evitar valores mágicos duplicados/divergentes entre arquivos.
    /// </summary>
    public static class NavMeshJobConstants
    {
        /// <summary>
        /// Número máximo de waypoints que um corredor (resultado do funnel) pode ter.
        /// Os buffers de corredor são flat arrays de tamanho AgentCapacity * MaxCorridorPoints,
        /// então isso afeta diretamente o consumo de memória. 64 é generoso pra maioria dos casos;
        /// se o caminho for mais longo que isso, o status vira PartialCorridor (ver PathStatus).
        /// </summary>
        public const int MaxCorridorPoints = 64;

        /// <summary>Unity suporta até 32 áreas de NavMesh (índices 0..31).</summary>
        public const int MaxNavMeshAreas = 32;
    }
}
