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

        /// <summary>
        /// O Transform do agente foi movido por fora da API deste pacote (física de
        /// knockback, root motion de animação, cutscene, etc.) desde a última vez que
        /// AvoidanceAndMoveJob escreveu nele — a posição externa foi ADOTADA como novo
        /// ponto de partida da simulação (reclampada na malha, igual um Warp implícito) em
        /// vez de silenciosamente sobrescrita no próximo frame. Não é necessariamente um
        /// bug: é o comportamento esperado se o jogo mexe no Transform diretamente. Vale
        /// investigar só se aparecer sem uma causa conhecida (ver README, "Movendo o
        /// Transform por fora da API").
        /// </summary>
        ExternalPositionAdopted = 3,

        /// <summary>
        /// O agente está tentando se mover (velocidade preferida — corredor, flow field ou
        /// override — acima de um piso configurável de MaxSpeed) mas sua posição não avança
        /// de verdade há mais de <c>NoProgressThresholdSeconds</c> segundos seguidos. Ao
        /// contrário de LostNavMesh (nenhum triângulo achado pra posição), aqui o
        /// ClampToNavMesh continua achando um triângulo válido normalmente — o agente
        /// simplesmente não consegue atravessar na direção que está tentando ir (ex.: uma
        /// borda real do NavMesh bem onde o flow field/corredor está mandando ele ir,
        /// avoidance empurrando de volta pro mesmo ponto todo frame). Sem essa detecção, um
        /// agente nessa situação fica com MovementFault = None pra sempre — velocidade
        /// calculada não-zero, posição idêntica frame após frame, sem nenhum sinal de que
        /// algo está errado. Não é auto-recuperável pelo pacote (a causa pode ser legítima —
        /// um buraco real na malha, uma multidão travando um gargalo) — é só visibilidade;
        /// quem consome decide o que fazer (redirecionar, repath por outro destino, etc.).
        /// </summary>
        NoProgress = 4,
    }
}
