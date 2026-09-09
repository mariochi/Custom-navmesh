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

        /// <summary>
        /// Início e fim são válidos, mas não existe caminho conectando os dois (ilhas de
        /// NavMesh diferentes, etc.) E o A* não conseguiu nem chegar perto o bastante do
        /// destino pra montar um corredor de melhor esforço (ver BestEffort) — corredor
        /// vazio, agente fica parado. Na prática raro: normalmente BestEffort cobre esse
        /// caso; isso só acontece se o triângulo de início não tiver NENHUM vizinho
        /// alcançável (agente isolado num triângulo solto).
        /// </summary>
        NoPath = 3,

        /// <summary>Início ou fim não encontraram nenhum triângulo próximo (fora do alcance do NavMesh).</summary>
        Invalid = 4,

        /// <summary>Agente sendo movido por um flow field de grupo (MoveGroupWithFlowField) em vez de corredor individual.</summary>
        FlowField = 5,

        /// <summary>
        /// Início e fim pertencem a componentes desconectados do grafo (ilhas de NavMesh
        /// diferentes — ex.: destino do outro lado de um buraco/parede sem ligação), mas o
        /// corredor gerado leva o agente até o ponto alcançável mais próximo do destino
        /// real, dentro da própria ilha de início (equivalente a NavMeshPathStatus.
        /// PathPartial do NavMeshAgent nativo). Diferente de PartialCorridor: aqui o
        /// CAMINHO em si não chega no destino pedido — não é só truncamento de buffer.
        /// </summary>
        BestEffort = 6,
    }
}
