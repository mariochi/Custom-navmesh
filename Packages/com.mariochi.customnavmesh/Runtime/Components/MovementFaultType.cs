namespace CustomNavMesh
{
    /// <summary>
    /// Falha de movimento detectada no frame atual pra um agente (diferente de PathStatus,
    /// que é sobre o RESULTADO de um pedido de caminho — isso é sobre o próprio pipeline de
    /// movimento/avoidance ter produzido algo inválido). Escrito por AvoidanceAndMoveJob,
    /// lido pelo NavMeshJobManager pra logar e desenhar no gizmo de debug.
    /// </summary>
    public enum MovementFaultType : byte
    {
        /// <summary>Nada de errado neste frame.</summary>
        None = 0,

        /// <summary>
        /// A velocidade calculada (avoidance, flow field ou corredor) deu NaN/Infinity —
        /// zerada antes de integrar a posição, pra não propagar a corrupção. Causa mais
        /// provável: triângulo degenerado (quase colinear) no NavMesh baked, ou uma
        /// combinação extrema de posição/velocidade relativa no cálculo de avoidance.
        /// </summary>
        InvalidVelocity = 1,

        /// <summary>
        /// ClampToNavMesh não achou NENHUM triângulo pra essa posição (nem o cache+vizinhos,
        /// nem a busca completa no grid espacial) — o agente foi mantido na última posição
        /// confirmada válida em vez de aceitar a posição nova (que pode estar fora do NavMesh
        /// ou corrompida). Se isso persistir por muitos frames seguidos, o agente ficou
        /// genuinamente preso fora da malha (não é só um solavanco de 1 frame).
        /// </summary>
        LostNavMesh = 2,
    }
}
