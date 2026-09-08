# CustomNavMesh — pathfinding + avoidance multithreaded via Job System

Sistema próprio de agentes de navegação que lê a triangulação gerada pelo bake padrão
do Unity (`NavMesh.CalculateTriangulation()`) e resolve pathfinding, avoidance local e
movimento inteiramente em `Unity.Jobs` + `Burst`, evitando o gargalo de main thread do
`NavMeshAgent`/`NavMesh.CalculatePath` nativos.

**Importante:** isso não substitui o bake do NavMesh. Você continua usando o pipeline
normal do Unity (`Window > AI > Navigation` ou um `NavMeshSurface` do pacote
`com.unity.ai.navigation`) pra gerar a malha. Este sistema só **lê** essa malha já
pronta e faz as consultas de caminho/movimento fora da main thread.

## Instalação em outro projeto

Este é um pacote UPM (`Packages/com.mariochi.customnavmesh` dentro do repositório
[Custom-navmesh](https://github.com/mariochi/Custom-navmesh) — o resto do repo é só o
projeto de teste/desenvolvimento). Duas formas de instalar num projeto novo:

- **Via Git URL** (recomendado — Package Manager → `+` → *Add package from git URL*):
  ```
  https://github.com/mariochi/Custom-navmesh.git?path=Packages/com.mariochi.customnavmesh
  ```
  O parâmetro `?path=` aponta pro subdiretório do pacote dentro do repo (o repo inteiro
  não é o pacote). Pra travar numa versão/commit específico, acrescente `#<hash-ou-tag>`
  no final da URL.
- **Copiando a pasta**: copie `Packages/com.mariochi.customnavmesh/` inteira pra dentro
  da pasta `Packages/` do outro projeto — o Editor reconhece automaticamente qualquer
  pasta com `package.json` ali dentro como pacote embutido, sem precisar editar o
  `manifest.json`.

Dependências (`com.unity.burst`, `com.unity.collections`, `com.unity.mathematics`) são
resolvidas automaticamente pelo Package Manager a partir do `package.json`.

## Setup rápido

1. Faça o bake do NavMesh normalmente (Navigation window ou `NavMeshSurface.BuildNavMesh()`).
2. Crie um GameObject vazio na cena e adicione o componente `NavMeshJobManager`
   ([Runtime/Components/NavMeshJobManager.cs](Runtime/Components/NavMeshJobManager.cs)). Só pode existir um por cena.
3. Ajuste `Agent Capacity` no inspector pra um valor >= ao número máximo de agentes que
   você vai ter simultâneos (os buffers são alocados com esse tamanho fixo no `Awake`).
4. Em cada personagem que deve navegar, adicione `CustomNavMeshAgent`
   ([Runtime/Components/CustomNavMeshAgent.cs](Runtime/Components/CustomNavMeshAgent.cs)) — **não** use o
   `NavMeshAgent` do Unity junto, os dois sistemas de movimento colidiriam.
5. Chame `agent.SetDestination(worldPos)` de qualquer script pra pedir um caminho.
   Leia `agent.Status`, `agent.Velocity`, `agent.HasArrived` pra acompanhar o progresso.
6. Se o pivot do seu modelo não estiver nos pés (ex.: capsule com pivot no centro), ajuste
   `Height` no inspector do `CustomNavMeshAgent` — é um deslocamento vertical aplicado só
   na posição final do Transform (equivalente ao `Base Offset` do `NavMeshAgent` padrão);
   a simulação (pathfinding/avoidance) continua rente à malha, sem esse offset.
7. Se o NavMesh for reconstruído em runtime (ex.: `NavMeshSurface.BuildNavMesh()` de novo
   depois de gerar terreno procedural), chame `NavMeshJobManager.Instance.RebuildGraph()`
   em seguida.

## Quando é seguro chamar a API (contrato de thread-safety)

**Leia isto antes de integrar `CustomNavMeshAgent`/`NavMeshJobManager` em código de
gameplay.** É uma restrição rígida do design, não uma sugestão de estilo — desrespeitar
resultou num crash real de thread-safety do Job System numa revisão do pacote já instalado
no jogo (`NavMeshMovement.Update()` lendo/escrevendo a API com o Job em voo).

Por baixo dos panos, `NavMeshJobManager` (`[DefaultExecutionOrder(-100)]`, sempre roda
antes dos scripts com ordem padrão) faz `Schedule()` **e** `Complete()` do Job do frame
dentro da **mesma chamada** do próprio `LateUpdate()` — depois que **todo** `Update()` da
cena já rodou. Isso significa que os `NativeArray`s por trás da API (`Velocity`,
`IsOnNavMesh`, `Status`, `SetVelocityOverride`, `SetAvoidanceOverride`, `Radius`/`Height`/
`MaxSpeed`, `SetDestination`, `Warp`, `Pause`/`Resume` etc.) nunca estão com um Job em voo
durante o `Update()` de nenhum script — é a única janela do frame com essa garantia.

**Regra prática: chame a API do `CustomNavMeshAgent`/`NavMeshJobManager` de dentro de
`Update()` (de qualquer script, ordem de execução padrão ou não — a do manager sempre
executa primeiro). Nunca de `LateUpdate()` nem de `FixedUpdate()`.**

- `LateUpdate()` de outro script: já não trava mais o Editor com Safety Checks (o
  `Schedule`/`Complete` do manager está isolado dentro do `LateUpdate()` dele, que sempre
  roda primeiro), mas o `ScheduleFrameJobs()` deste frame **já rodou** antes do seu
  `LateUpdate()` ser chamado — então `SetDestination`/`Warp`/`Pause` chamados ali só
  entram em vigor no Job do **próximo** frame (atraso de 1 frame, silencioso).
- `FixedUpdate()`: roda **antes** do `Update()` do manager nesse mesmo frame, então
  qualquer coisa setada ali é apagada pelo `ExpirePerFrameOverrides()` do próprio
  `Update()` do manager (ver abaixo) antes de qualquer Job ler — na prática, o mesmo
  problema de overrides "erased before consumption" que motivou este redesenho todo.
- `Awake()`/`Start()`/corrotinas: seguros pra chamadas que não expiram por frame
  (`SetDestination`, `Warp`, `Pause/Resume`, `SetRadius/SetMaxSpeed/SetHeight`,
  `MoveGroupWithFlowField`, `RepathAllAgents`, `NotifyNavMeshChanged` — o pior caso é um
  atraso de até 1 frame se caírem antes do primeiro `LateUpdate()` agendar). **Não** são
  seguros pra `SetVelocityOverride`/`SetAvoidanceOverride` — ver próximo bullet.

**Caso especial: `SetVelocityOverride`/`SetAvoidanceOverride`.** Esses dois são "válidos
só neste frame" por design — o manager zera os dois logo no início do **seu próprio**
`Update()` (`ExpirePerFrameOverrides()`, antes de qualquer script de gameplay rodar o
`Update()` dele nesse frame), justamente pra permitir que você chame de novo todo frame
sem precisar de `Clear...` explícito. Consequência: só chamar de dentro de `Update()`
garante que o valor sobreviva até o `LateUpdate()` do mesmo frame agendar o Job. Chamado
de `Awake()`/`Start()`/`FixedUpdate()`, o valor é apagado pelo próprio
`ExpirePerFrameOverrides()` antes de qualquer Job existir pra lê-lo — nunca surte efeito.
Chamado de `LateUpdate()`, o Job deste frame já foi agendado sem ele, e ele é apagado no
próximo `Update()` antes do Job seguinte rodar — também nunca surte efeito. `Update()` é
literalmente a única janela em que esses dois funcionam.

Resumindo numa frase: **se é `Update()`, está seguro e some no máximo em 0 frames de
atraso; qualquer outra fase, ou funciona com atraso de 1 frame (chamadas normais) ou nunca
funciona (overrides por frame).**

## Flow field (movimento de grupo)

Pra grupos grandes de agentes convergindo pro **mesmo destino** (comando de RTS: seleciona
N unidades, clica um ponto), pedir um `SetDestination` individual por agente significa
um A* completo por agente — desperdício quando todos vão pro mesmo lugar. Um flow field
resolve isso ao contrário: parte do destino e expande por Dijkstra até **todos** os
triângulos do NavMesh de uma vez só, produzindo uma direção de fluxo por triângulo. Esse
campo é compartilhado — qualquer nº de agentes só consulta "qual a direção do triângulo
onde eu estou", sem recalcular nada por agente.

```csharp
NavMeshJobManager.Instance.MoveGroupWithFlowField(selectedAgents, targetPosition);
```

- `agents`: array de `CustomNavMeshAgent` (precisam já estar registrados, ou seja,
  `enabled` e com `AgentIndex` válido — normal se já estão na cena).
- `destination`: ponto no mundo; se cair fora da área coberta pelo NavMesh, **ou** cair
  numa área que `areaMask` não permite, a chamada loga um aviso e não faz nada (retorna
  `false`) — antes só a primeira checagem existia, um destino tecnicamente sobre o NavMesh
  mas numa área bloqueada (ex.: "água" fora de `areaMask`) passava batido e o Dijkstra
  rodava a partir de um triângulo que ele mesmo trataria como inacessível.
- `areaMask` (opcional): igual ao `AreaMask` do `CustomNavMeshAgent`, filtra quais áreas
  do NavMesh o campo pode atravessar.
- `keepFormation` (opcional, default `true`): em vez de todo mundo mirar o mesmo ponto
  exato (o que aglomera o grupo num círculo apertado ao chegar), cada agente recebe seu
  próprio ponto de chegada = destino + a posição relativa dele ao centróide do grupo **no
  momento da chamada** — preserva o formato/espaçamento que o esquadrão já tinha, não
  impõe uma formação pré-definida. Esse deslocamento é rotacionado pra acompanhar a nova
  direção de deslocamento (estimada pela velocidade média atual do grupo; se o grupo
  estiver parado, não rotaciona) e reprojetado no NavMesh (a formação pode cair fora da
  malha/contra uma parede perto do destino). O flow field compartilhado ainda guia todo
  mundo durante o trajeto inteiro — só perto do alvo (`Flow Field Arrive Distance`) cada
  agente diverge pro seu ponto individual, então o grupo continua coeso pelo caminho e só
  "abre" pra formação nos últimos metros. Passe `false` pra voltar ao comportamento
  antigo (todo mundo no mesmo ponto exato).

Mecânica interna, se for mexer/entender o código:

- Cada slot do pool (`Max Flow Fields` no inspector, default 8) guarda um campo completo
  (`Directions`/`Distance`, achatados em `maxFlowFields * TriangleCount`). Chamar
  `MoveGroupWithFlowField` de novo com um destino diferente ocupa outro slot; se todos os
  8 estiverem em uso (por outros grupos que ainda não chegaram), a chamada loga erro e
  falha — suba `Max Flow Fields` se seu jogo tem muitos comandos de grupo simultâneos.
  Cada slot é contado por referência (quantos agentes o estão usando) e é liberado
  automaticamente assim que o último agente sai dele (chega ao destino, recebe
  `SetDestination` individual, ou é desregistrado).
- **Mutuamente exclusivo com `SetDestination`** por agente: um agente só está num modo
  ou no outro. Chamar `SetDestination` nele tira do flow field (volta a seguir corredor
  A*/funnel individual) no frame seguinte; chamar `MoveGroupWithFlowField` de novo o move
  pro novo campo. `agent.IsUsingFlowField` e `agent.Status == PathStatus.FlowField`
  contam qual modo está ativo.
- **Handoff híbrido perto do alvo.** Ao entrar no raio de `Flow Field Arrive Distance`
  (default 2 unidades de distância-ao-longo-do-campo, não linha reta),
  `CheckFlowFieldArrivals` promove o agente automaticamente pro pipeline individual —
  chama `SetDestination(pontoDeFormaçãoDele)` por baixo dos panos, que aciona o mesmo
  mecanismo de `SetDestination` manual (libera o slot do flow field, enfileira um repath
  individual). Ou seja: o flow field resolve o trajeto longo em grupo (barato, um cálculo
  pra todo mundo), e a reta final até o ponto exato de formação usa A* + funnel completo
  (já validado sem zigue-zague), respeitando a malha em vez de uma linha reta ingênua até
  o ponto. Combina o melhor dos dois: throughput do flow field pra distância, precisão do
  corredor individual pra aproximação final.
- **Quando vale a pena**: o Dijkstra roda sobre o grafo **inteiro** alcançável a partir do
  alvo, então o custo não cai mesmo se só 1 agente usar o campo — só compensa sobre
  `SetDestination` individual quando **vários** agentes convergem pro mesmo ponto. Pra um
  agente sozinho, ou poucos agentes com destinos diferentes, use `SetDestination`.
- ~~**Zigue-zague no flow field.**~~ Resolvido em duas etapas. Primeiro, a direção de cada
  triângulo apontava pro centróide do vizinho de menor distância; trocado por apontar pro
  ponto médio da aresta compartilhada (portal) — mesma ideia por trás do funnel. Isso
  ajudou mas não eliminou: a direção continuava sendo um vetor CONSTANTE por triângulo,
  escolhido discretamente entre 3 vizinhos — uma oscilação real (não só visual) sempre
  que a escolha mudava de um triângulo pro seguinte, mais visível perto do alvo (onde
  vários caminhos convergem). A correção definitiva foi trocar a derivação da direção:
  `ComputeFlowFieldJob` agora propaga a distância de cada triângulo pros seus 3 vértices
  (mínimo entre os triângulos incidentes) e reconstrói o **gradiente do interpolante
  linear** que passa pelos 3 valores — como triângulos vizinhos compartilham 2 dos 3
  vértices, a direção resultante concorda muito mais entre eles (mesma ideia de normal
  suavizada por vértice vs. normal por face).
- ~~**Zigue-zague residual perto do alvo / convergência de grupo.**~~ Resolvido —
  `AvoidanceAndMoveJob` aplicava a velocidade desejada direto, sem suavização; qualquer
  mudança abrupta de direção-alvo (cruzar de triângulo, vários agentes mirando o mesmo
  ponto exato perto do destino) aparecia como solavanco. `Steering Acceleration Factor`
  (no `NavMeshJobManager`, default 10× MaxSpeed/s) agora limita a variação de velocidade
  por frame; 0 desliga e volta ao comportamento antigo (salto direto).
- Avoidance entre agentes continua rodando normalmente por cima do flow field (mesmo
  `AvoidanceAndMoveJob`, só a fonte da velocidade preferida muda).

## Controle de movimento por agente

### Pause / Resume

```csharp
agent.Pause();   // trava a busca ativa do corredor/flow field — equivalente a NavMeshAgent.isStopped = true
agent.Resume();  // retoma exatamente de onde parou
bool paused = agent.IsPaused;
```

Diferente de `Stop()`: o corredor/flow field **não são descartados** (`Stop()` os zera). Um
agente pausado continua participando do avoidance normalmente — outros agentes o veem
como obstáculo, e ele reage se for empurrado (só a busca ativa por um alvo é que para).
Uso típico: travar um bot durante uma animação de ataque corpo-a-corpo e retomar o mesmo
destino depois, sem recalcular caminho.

### Warp

```csharp
bool ok = agent.Warp(worldPosition); // reposiciona instantaneamente, sem interpolar
```

Pra respawn ou pouso pós-movimento-forçado. Acha o triângulo mais próximo, zera
velocidade, descarta corredor/flow field/destino atual. `false` se o ponto está fora da
área coberta pelo NavMesh (nada muda nesse caso). Chame `SetDestination` de novo depois
se quiser que o agente continue andando pra algum lugar.

### Avoidance por agente

```csharp
agent.IgnoreAvoidance = true; // permanente até trocar de novo — equivalente a "No Obstacle Avoidance"

agent.SetAvoidanceOverride(neighborQueryRadius: 6f, timeHorizon: 3f); // válido só ESTE frame
agent.ClearAvoidanceOverride(); // normalmente desnecessário — some sozinho se parar de chamar
```

- **`IgnoreAvoidance`**: o agente atravessa outros agentes sem desviar (mas outros ainda
  desviam dele, já que a posição dele continua entrando no cálculo de avoidance dos
  vizinhos). Bom pra unidades grandes/chefes que não devem ser bloqueados pela própria tropa.
- **`SetAvoidanceOverride`**: sobrescreve `Neighbor Query Radius`/`Avoidance Time Horizon`
  globais só pra esse agente, só pelo frame atual — chame de novo todo frame enquanto
  quiser mantê-lo ativo (ex.: um trigger de zona chamando isso a cada `Update` enquanto o
  agente estiver dentro dela). Pare de chamar e ele volta ao valor global sozinho, sem
  precisar de `ClearAvoidanceOverride` explícito. **Atenção**: o raio de busca de vizinhos
  usa uma janela fixa de células 3x3 (`Neighbor Cell Size`, global) — um override de raio
  muito maior que ~1.5× `Neighbor Cell Size` não vai enxergar vizinhos além dessa janela;
  se precisar de raios bem maiores, suba `Neighbor Cell Size` também.
- **`Vertical Avoidance Range`** (global, no inspector do `NavMeshJobManager`, default 2):
  diferença de altura (Y) acima da qual dois agentes deixam de se enxergar pra avoidance,
  mesmo próximos em XZ. Sem isso, um agente em cima de uma muralha/ponte e outro embaixo
  dela se desviavam um do outro como se estivessem no mesmo plano (todo o cálculo de
  avoidance era feito só em XZ). Ajuste pra cobrir a altura típica de um agente — cobre
  demais e volta a juntar andares/pontes diferentes; cobre de menos e ignora vizinhos numa
  rampa suave que deveriam se enxergar. Não é por agente (ainda) — é um único valor global.

### Velocidade explícita (strafe / dodge / knockback / step)

```csharp
agent.SetVelocityOverride(direction * speed); // válido só ESTE frame, mesmo contrato de SetAvoidanceOverride
agent.ClearVelocityOverride(); // normalmente desnecessário — some sozinho se parar de chamar
```

Substitui a busca ativa de corredor/flow field por uma velocidade explícita, pra ações de
combate que setavam `NavMeshAgent.velocity` direto no sistema padrão do Unity (strafe,
dodge, knockback, "andar pra frente" numa animação de ataque). O agente continua dentro do
sistema — ainda clampado na malha (`ClampToNavMesh` roda incondicionalmente, não atravessa
parede), ainda sofre avoidance dos vizinhos (levemente desviado se for atravessar outro
agente) — mas pula o clamp de `MaxSpeed` (knockback precisa poder exceder a velocidade
normal de corrida) e a suavização de aceleração (`Steering Acceleration Factor`): é pra ser
instantâneo, sem rampa, igual `.velocity =` direto era.

**Tem prioridade sobre `Pause()`**: um agente pausado ainda se move se isso for chamado
nele — é assim que um ataque corpo-a-corpo consegue pausar o corredor (trava a busca ativa)
e empurrar o personagem pra frente no mesmo frame. Não mexe em `CorridorCursor`/flow
field — quando você parar de chamar, o agente retoma o corredor de onde estava, exatamente
como acontece com `Pause()`/`Resume()`.

### Raio/altura/velocidade ao vivo

`Radius`, `Height`, `MaxSpeed` e `WaypointReachDistance` agora propagam pro job
imediatamente ao serem trocados em runtime (antes, só eram lidos uma vez no registro do
agente — trocar `Radius` pra simular agachar, por exemplo, não tinha efeito nenhum no
avoidance). Nada muda na API — continuam sendo as mesmas propriedades de sempre:

```csharp
agent.Radius = crouching ? 0.3f : 0.5f; // já reflete no avoidance a partir do próximo frame
```

### Progresso do caminho

```csharp
float remaining = agent.RemainingDistance; // soma dos segmentos entre o waypoint atual e o fim do corredor
bool pending = agent.IsPathPending;        // true enquanto o pedido de repath está na fila (budget de Max Path Requests Per Frame)
bool onMesh = agent.IsOnNavMesh;           // leitura O(1) — não faz nenhuma consulta nova, só reflete o triângulo já rastreado por frame
```

`RemainingDistance` no modo flow field é uma aproximação (distância-ao-longo-do-campo até
o triângulo de destino, não o caminho exato até o ponto de formação do agente) — exata só
no modo corredor individual. `IsOnNavMesh` é o equivalente a `NavMeshAgent.isOnNavMesh`,
mas de graça: como todo agente já rastreia seu triângulo atual por frame (pro clamp de
superfície), essa propriedade só olha esse dado — nada de `NavMesh.SamplePosition` síncrono
por trás, então é seguro chamar todo frame por agente sem reintroduzir o gargalo de main
thread que o pacote existe pra evitar. Só fica `false` depois de um `MovementFaultType.LostNavMesh`
persistente (ver "Diagnóstico de travamentos silenciosos") ou antes do primeiro frame do
agente rodar.

## Rebuild quando o NavMesh muda

Se o jogo faz carving em runtime (`NavMeshObstacle`, portões, barreiras mágicas, etc.), o
grafo do pacote precisa ser reconstruído — mas isso **não acontece sozinho**. Não existe
um jeito confiável de detectar "o NavMesh mudou" sem o jogo avisar: o candidato óbvio,
`NavMesh.onPreUpdate`, dispara a cada tick do subsistema de navegação do Unity — ou seja,
dispara mesmo quando nada mudou, então não dá pra diferenciar "mudou" de "só rodou o
tick" só com esse evento (uma versão anterior deste README/código tentava usar isso com
debounce e o mecanismo ficava permanentemente travado: o evento reempurrava o prazo do
debounce antes da janela conseguir fechar, então o rebuild nunca disparava de verdade —
obrigado a quem revisou e pegou isso).

O gatilho é explícito — chame quando **souber** que algo mudou (o próprio script do
portão/obstáculo, no momento em que ele liga/desliga):

```csharp
NavMeshJobManager.Instance.NotifyNavMeshChanged();
```

- Debounced (`Auto Rebuild Debounce`, default 0.25s): várias chamadas em sequência rápida
  (vários obstáculos mudando juntos) viram uma única reconstrução.
- `Auto Rebuild On NavMesh Change` (default `true`) é um master switch — desligado, as
  chamadas a `NotifyNavMeshChanged()` são ignoradas (útil pra desligar tudo de uma vez em
  debug/profiling sem precisar tirar a chamada do código do jogo).
- Por baixo dos panos, chama `RebuildGraph()` + `RepathAllAgents()` — as duas peças
  continuam expostas separadamente se você quiser controle mais fino:

```csharp
NavMeshJobManager.Instance.RebuildGraph();               // reconstrói o grafo a partir do NavMesh atual (síncrono, sem debounce)
NavMeshJobManager.Instance.RepathAllAgents();             // força recálculo de quem tem destino ativo (modo corredor)
NavMeshJobManager.Instance.RepathAgentsNear(point, 10f);  // versão barata: só quem está perto do que mudou
```

`RebuildGraph()` sozinho **não** força ninguém a recalcular — só invalida os índices de
triângulo internos. Sem `RepathAllAgents()`/`RepathAgentsNear()` em seguida, um agente
pode continuar seguindo um corredor calculado em cima da topologia antiga (ex.: atravessando
um portão que acabou de fechar) até o handoff natural (chegar ao fim do corredor ou pedir
um novo destino). `RepathAllAgents`/`RepathAgentsNear` ignoram agentes sem destino ativo
(nada pra recalcular) e agentes em flow field (um `RebuildGraph()` já tira todo mundo do
flow field automaticamente — se o grupo ainda precisa se mover, chame
`MoveGroupWithFlowField` de novo).

## Debug visual

O `NavMeshJobManager` tem cinco toggles de Gizmo (Scene view, em Play mode):

- **`Draw Status Gizmos`** — uma esfera colorida sobre cada agente com o `PathStatus`
  atual: cinza = nunca pediu path, verde = `Success`, amarelo = `PartialCorridor`,
  vermelho = `NoPath` (início/fim válidos mas sem rota entre eles), magenta = `Invalid`
  (início ou fim caiu fora da área coberta pelo NavMesh), ciano = `FlowField` (seguindo
  campo de grupo). **É o primeiro lugar pra olhar se um agente não se move** — se ele
  estiver vermelho ou magenta, o problema é o destino pedido, não o pipeline de movimento.
  Uma esfera **preta sólida** por cima de qualquer uma dessas cores sinaliza um
  `MovementFault` no frame atual (ver seção "Diagnóstico de travamentos silenciosos" abaixo)
  — isso é mais grave que um status de path ruim, é o próprio pipeline de movimento tendo
  produzido algo inválido.
- **`Draw NavMesh Gizmo`** — wireframe da triangulação que o sistema está enxergando
  (útil pra confirmar que a área que você imagina baked realmente está sendo lida).
- **`Draw Corridor Gizmos`** — linhas amarelas com o corredor calculado de cada agente
  (vazio pros agentes em modo flow field, que não usam corredor).
- **`Draw Flow Field Gizmo`** + **`Flow Field Gizmo Slot`** — uma seta por triângulo com
  a direção de fluxo do slot escolhido (0-indexado, na ordem em que os grupos foram
  criados e ainda não chegaram/trocaram de destino). Também desenha uma esfera branca no
  destino bruto do comando e uma esfera magenta no ponto individual de cada agente
  daquele slot — dá pra ver visualmente o quanto a formação está "abrindo" o alvo.

## Diagnóstico de travamentos silenciosos

Sintoma raro reportado em produção: depois de algumas horas rodando, algum agente
esporádico simplesmente para de se mover — sem nenhum erro no Console. Causa suspeita:
`NaN`/`Infinity` se infiltrando na velocidade (triângulo degenerado no NavMesh baked, ou
uma combinação extrema de posição/velocidade relativa no avoidance). O grave de `NaN` é
que ele nunca lança exceção e **toda comparação `<` com `NaN` dá falso** — então a busca
de "qual triângulo está mais perto dessa posição" nunca encontra nada pra uma posição
`NaN`, o agente perde a referência de triângulo pra sempre, e nada em lugar nenhum
detectava isso antes desta seção existir.

O que foi adicionado (`AvoidanceAndMoveJob` + `MovementFaultType`):

- **Detecção**: depois de calcular a velocidade (avoidance + flow field/corredor), checa
  `NaN`/`Infinity` explicitamente. Se achar, zera a velocidade desse frame em vez de
  deixar propagar, e sinaliza `MovementFaultType.InvalidVelocity`.
- **Autorrecuperação de posição**: `ClampToNavMesh` agora, se não achar NENHUM triângulo
  (nem no cache local, nem na busca completa do grid), **não aceita** a posição nova
  (pode estar corrompida ou fora da malha) — mantém o agente na última posição confirmada
  válida do início do frame, e preserva a última referência de triângulo válida (não
  sobrescreve com -1) pra dar ao próximo frame a melhor chance de reconectar. Sinaliza
  `MovementFaultType.LostNavMesh`.
- **Autorrecuperação de velocidade suavizada**: `SmoothVelocity` também checa se a
  velocidade do frame anterior já estava com `NaN` (caso a corrupção tenha acontecido
  antes desse fix existir) e pula a suavização nesse caso, pra não misturar `NaN` com um
  valor limpo.
- **Visibilidade**: `NavMeshJobManager` loga um `Debug.LogWarning` (uma vez por ocorrência,
  não todo frame) identificando o agente e o tipo de falha, e desenha uma esfera preta
  sólida sobre ele (`Draw Status Gizmos`) enquanto o `MovementFault` persistir.

Se isso disparar de novo: o log te diz exatamente qual agente e qual dos dois tipos —
`InvalidVelocity` aponta pra matemática do avoidance/campo de fluxo produzindo `NaN`;
`LostNavMesh` aponta pra uma posição que saiu longe demais da malha num frame só (ex.:
empurrão de avoidance grande demais, ou teleporte externo do Transform sem passar por
`SetDestination`/`MoveGroupWithFlowField`). Isso não deveria mais travar o agente pra
sempre — ele fica visível e, na maioria dos casos, se recupera sozinho em poucos frames;
se um agente ficar com a esfera preta por muito tempo seguido, é sinal de que está
genuinamente preso fora do NavMesh (não um solavanco de 1 frame), e vale investigar a
geometria da área onde ele está.

## Arquitetura

```
NavMesh.CalculateTriangulation()  (main thread, só no bake/rebuild)
        │
        ▼
NavMeshGraphBuilder ──► NavMeshGraph (NativeArrays: vértices, triângulos, vizinhos, custo por área)
        │
        ▼
NavMeshSpatialGrid  (broad-phase XZ: célula -> triângulos que a tocam)
        │
        ▼  (por frame, em Jobs)
FindPathsBatchJob (IJobParallelFor)     — A* sobre o grafo de triângulos + funnel/string-pulling
ComputeFlowFieldJob (IJob)              — Dijkstra a partir do destino, sob demanda (MoveGroupWithFlowField)
BuildAgentSpatialHashJob (IJobParallelFor) — grid de agentes pra consulta de vizinhos
AvoidanceAndMoveJob (IJobParallelForTransform) — avoidance local + integra posição + clamp na malha
                                            (por agente: segue corredor OU flow field, nunca os dois)
```

Pipeline por frame (`NavMeshJobManager`, ver também "Quando é seguro chamar a API" acima):

- **`Update()`**: completa o `JobHandle` agendado no `LateUpdate()` anterior (deveria já
  estar pronto — é só uma garantia idempotente), copia posição/velocidade resolvidas pro
  array que a API lê, loga faults, expira os overrides por frame
  (`ExpirePerFrameOverrides`), checa chegadas de flow field e coleta os agentes que
  pediram um novo destino desde o último frame (respeitando `Max Path Requests Per
  Frame`). **Não agenda o Job aqui** — só depois que todo `Update()` da cena rodou.
- **`LateUpdate()`**: agenda **e** completa o `JobHandle` do frame na mesma chamada
  (`ScheduleFrameJobs()` + `Complete()`). Como `NavMeshJobManager` tem
  `[DefaultExecutionOrder(-100)]`, esse `LateUpdate()` roda antes do de qualquer outro
  script — ou seja, o Schedule/Complete inteiro acontece isolado, sem nenhum código de
  gameplay rodando no meio. É essa janela (não mais overlap entre Schedule e Complete)
  que garante que ler/escrever a API em `Update()` de qualquer script nunca colide com um
  Job em voo — trade-off deliberado: abrimos mão do overlap "Job rodando enquanto o resto
  do frame roda" (ganho secundário) pra eliminar uma violação real de thread-safety do Job
  System encontrada em revisão (script de gameplay lendo/escrevendo com o Job já agendado
  e ainda não completado). O ganho principal — resolver todos os agentes em paralelo
  entre si nos worker threads — continua intacto.

Cada agente é uma linha "densa" (índice compacto, sem buracos) nos NativeArrays do
manager, espelhando um `TransformAccessArray`. Remover um agente faz swap-back do
último índice ativo pro slot liberado (`CustomNavMeshAgent.AgentIndex` é atualizado
automaticamente quando isso acontece).

## Mapas grandes (ex.: 1000x1000)

O A* de `FindPathsBatchJob` **não tem raio de busca**: ele expande o grafo de triângulos
até achar o triângulo de destino (ou esgotar o grafo), então pedir um caminho de um canto
ao outro de um mapa 1000x1000 já funciona sem nenhuma configuração especial. O que precisa
de calibração pra esse tamanho de mapa é o **broad-phase**, não o pathfinding em si:

- **`Triangle Grid Cell Size`** (no `NavMeshJobManager`) — tamanho de célula do grid usado
  por `NavMeshQueryUtil.FindNearestTriangle` pra achar rápido "em qual triângulo está essa
  posição" (usado no início/fim de todo path request e no clamp de superfície por frame).
  O default (4) foi pensado pra navmeshes pequenos/médios; num mapa 1000x1000 ele cria uma
  grade de ~250x250 células, que costuma ser overhead sem ganho se o navmesh for uma
  triangulação aberta/grosseira (poucos triângulos grandes cobrindo áreas enormes — comum
  em terrenos planos). Regra prática: comece em algo como **2-4× o tamanho médio de aresta
  dos triângulos** do seu navmesh, não uma fração do tamanho do mapa. Depois de compilar,
  olhe o `Debug.Log` que o `NavMeshJobManager` imprime em todo `RebuildGraph()` — ele
  mostra o nº de triângulos e o grid resultante, dá pra ajustar de olho nesse número (ou
  ler `NavMeshJobManager.Instance.TriangleCount` em runtime).
- **Nº de triângulos do navmesh** não é determinado pelo tamanho do mapa, e sim pelas
  configurações de bake do `NavMeshSurface` (`Voxel Size` / `Tile Size` / densidade de
  obstáculos). Um plano aberto de 1000x1000 pode virar poucas centenas de triângulos; um
  terreno cheio de obstáculos pode virar dezenas de milhares. Isso importa porque
  `FindPathsBatchJob` aloca arrays de escopo O(nº de triângulos) por request (ver
  limitação abaixo) — se o Console acusar navmesh com dezenas de milhares de triângulos e
  os requests de path ficarem caros, geralmente compensa mais aumentar o `Voxel Size` do
  bake (simplificar a malha) do que tentar compensar só no lado do nosso pathfinding.
- **`Max Path Requests Per Frame`** — se muitos agentes pedirem caminhos longos (que
  atravessam boa parte do mapa) no mesmo frame, cada request individual tende a visitar
  mais triângulos que um path curto; considere testar com o Profiler se o budget default
  (32) ainda é suficiente pro seu caso.
- **Tiles do NavMesh (`Vertex Weld Epsilon`).** Mapas grandes quase sempre são baked em
  múltiplos tiles (Recast/Detour). `NavMesh.CalculateTriangulation()` não garante índice
  de vértice compartilhado na costura entre tiles — sem tratar isso, `BuildTriangleAdjacencyJob`
  enxerga cada tile como uma ilha isolada e todo path que cruza uma fronteira de tile falha
  com `PathStatus.NoPath`, mesmo o NavMesh sendo visualmente contínuo. `NavMeshGraphBuilder`
  já solda vértices coincidentes (dentro de `Vertex Weld Epsilon`, default 1cm) antes de
  montar o grafo — isso resolve o caso comum. Se o `Debug.Log` do `RebuildGraph()` mostrar
  poucos/nenhum vértice soldado e `NoPath` persistir só nas fronteiras de tile, aumente
  `Vertex Weld Epsilon` (a costura pode ter uma folga maior que o default, dependendo do
  `Voxel Size` do bake).

## Limitações conhecidas / pontos de atenção

- ~~**AreaMask não validado no atalho de mesmo triângulo.**~~ Resolvido — quando início e
  fim de um `SetDestination` caíam no mesmo triângulo, `FindPathsBatchJob` retornava
  `Success` direto (atalho antes de qualquer expansão do A*), sem passar pelo
  `IsAreaAllowed` que os vizinhos expandidos já checavam normalmente. Um destino técnica-
  mente alcançável mas numa área bloqueada por `AreaMask` (ex.: "água") era aceito como se
  fosse válido. Agora o atalho valida a área de `startTri`/`endTri` antes de aceitar (ver
  [FindPathsBatchJob.cs](Runtime/Jobs/FindPathsBatchJob.cs)); o mesmo buraco existia (e foi
  fechado) pro alvo de `MoveGroupWithFlowField` (ver seção "Flow field" acima).
- ~~**`CorridorCursor` não resetava num repath.**~~ Resolvido — pedir um novo
  `SetDestination` gerava um corredor novo, mas o índice de progresso (`CorridorCursor`)
  do agente continuava de onde tinha parado no corredor ANTERIOR. Corredores mais curtos
  que o cursor herdado faziam o agente mirar um índice fora do corredor novo (efeito
  visual: andar colado numa parede/desviando estranho logo após um repath, em vez de seguir
  o waypoint 0 do caminho recém-calculado). `NavMeshJobManager` agora zera o cursor no
  mesmo momento em que monta o `PathRequest`, antes de agendar o Job.
- **Zigue-zague em degraus de escada.** `ClampToNavMesh` (o "onde estou na malha agora")
  testa o triângulo em cache + seus 3 vizinhos e fica com o mais próximo — sem nenhuma
  histerese, isso funciona bem em terreno normal (triângulos grandes, um vencedor óbvio),
  mas em degraus (triângulos pequenos e muito próximos entre si, várias transições de
  triângulo por metro) o "mais próximo" fica empatado entre o triângulo atual e um vizinho
  a cada frame só por ruído de sub-milímetro na posição — cada troca reprojeta a posição
  clampada (principalmente o Y) discretamente, e como a direção do frame seguinte é
  calculada a partir dessa posição, isso vira zigue-zague visível concentrado exatamente
  nas bordas dos degraus (a malha do funnel em si é imune a isso — `Funnel.TriArea2D`
  projeta tudo em XZ, ignorando Y). Corrigido com `Triangle Sticky Margin` (no inspector
  do `NavMeshJobManager`, default 0.02 = 2cm): um vizinho só substitui o triângulo atual
  se vencer por mais que essa margem de distância, não só por estar marginalmente mais
  perto — pequeno o bastante pra não atrapalhar uma transição real (que muda a posição por
  bem mais que isso conforme o agente anda). Se o zigue-zague em escadas persistir, suba
  um pouco esse valor; se agentes parecerem "grudar" um frame a mais que deveriam ao mudar
  de triângulo em terreno normal, abaixe. **Dica relacionada**: se os agentes também
  parecerem "pular" degraus ou tentar avançar vários de uma vez antes disso, verifique se
  `Waypoint Reach Distance` não está maior que a profundidade do degrau (o funnel gera
  cerca de 1 waypoint por degrau em escadas estreitas) — um valor grande demais dá o
  waypoint do degrau seguinte como "alcançado" antes do agente realmente estar lá.
- **Capacidade fixa.** `Agent Capacity` aloca os buffers uma vez no `Awake`. Registrar
  mais agentes que a capacidade loga um erro e o `CustomNavMeshAgent` fica desabilitado.
  Não há realloc dinâmico (de propósito, pra não ter que gerenciar containers em voo
  durante um Job) — ajuste o valor no inspector.
- **Avoidance é uma aproximação, não ORCA/LP completo.** `AvoidanceAndMoveJob` usa
  time-to-collision + correção de penetração, dividido 50/50 entre os dois agentes
  envolvidos (cada um roda o mesmo cálculo do seu lado). As contribuições de todos os
  vizinhos são somadas contra a velocidade preferida FIXA do agente (não uma velocidade
  sendo mutada vizinho a vizinho) e depois têm a MÉDIA tirada pelo nº de vizinhos — sem
  isso, um agente cercado de muitos vizinhos (multidão densa) somava correções sem limite
  e a ordem de processamento afetava o resultado, causando oscilação/zigue-zague visível
  em cenas com centenas de agentes aglomerados. `Crowd Push Damping` (default 0.3) reduz
  a força do empurrão de sobreposição quando os dois agentes envolvidos já andam na mesma
  direção (marchando juntos — o caso de um grupo grande convergindo no mesmo flow field,
  que fica naturalmente mais apertado que `combinedRadius` por boa parte do trajeto sem
  estar de fato "colidindo"); colisões de frente/cruzadas continuam com empurrão cheio
  (1 = desliga esse amortecimento, comportamento antigo). Ainda não é a solução ótima
  (linear programming sobre half-planes) da RVO2/ORCA original — funciona bem pra
  densidades normais de jogo, mas se ainda notar oscilação em multidões muito densas/
  gargalos apertados mesmo com esse ajuste, o próximo passo é trocar o corpo de
  `AvoidanceAndMoveJob.ApplyAvoidance` por um solver de LP 2D de verdade — o resto do
  pipeline não muda.
- ~~**Winding do funnel.**~~ Resolvido — `Funnel.GetSharedEdge` estava com `left`/`right`
  trocados, o que fazia o corredor sair em zigue-zague em vez de reto (o funil "esticava"
  errado em quase todo portal). Convenção corrigida e confirmada contra o winding real do
  NavMesh do Unity.
- **Clamp de superfície é best-effort.** `AvoidanceAndMoveJob` mantém um triângulo em
  cache por agente e testa ele + os 3 vizinhos a cada frame antes de cair pro grid
  espacial completo; isso cobre bem movimento normal (inclusive rampas/escadas), mas um
  teleporte grande num único frame pode custar uma busca completa no grid naquele frame.
- **Pool de flow fields tem memória proporcional a `Max Flow Fields × TriangleCount`.**
  Cada slot guarda `Directions` (float3, 12 bytes) + `Distance` (float, 4 bytes) por
  triângulo. Com o default (8 slots) num navmesh de 5.000 triângulos isso é só ~640KB,
  mas em navmeshes muito grandes vale baixar `Max Flow Fields` se não for usar muitos
  grupos simultâneos.
- **Off-mesh links não são suportados.** `NavMesh.CalculateTriangulation()` só devolve a
  superfície triangulada; links (saltos, teleportes de NavMeshLink) não entram no grafo.
  Se precisar, teria que ser adicionado como um passo à parte (aresta extra artificial
  entre dois triângulos não-adjacentes).
- ~~**Sem `asmdef` próprio.**~~ Resolvido — o código virou um pacote UPM embutido
  (`Packages/com.mariochi.customnavmesh`) com `CustomNavMesh.Runtime.asmdef` próprio,
  isolado do `Assembly-CSharp` do projeto host.
- **Não testado dentro de uma sessão do Editor por mim** (este ambiente não tem o Unity
  Editor disponível pra compilar/rodar) — toda a validação de correção depende de análise
  estática cuidadosa mais o teste real que vocês fazem no Editor. `package.json` declara
  `"unity": "2021.3"` como piso (baixado de `6000.3`, que era otimista demais pro projeto
  real, que roda **Unity 2022.3.62f2**) — as APIs de Job System/Burst/Collections/
  Mathematics usadas aqui são estáveis desde bem antes disso, mas esse piso ainda não foi
  confirmado compilando de verdade numa 2021.3; se notar qualquer erro de compilação
  específico de versão, me manda a mensagem que eu ajusto o piso declarado ou o código.

## Arquivos

| Arquivo | Papel |
|---|---|
| `package.json` | Manifesto UPM (nome, versão, dependências). |
| `Runtime/CustomNavMesh.Runtime.asmdef` | Assembly Definition do pacote (referencia Burst/Collections/Mathematics). |
| `Runtime/NavMeshJobConstants.cs` | Constantes compartilhadas (`MaxCorridorPoints`, `MaxNavMeshAreas`). |
| `Runtime/Data/NavMeshGraph.cs` | Struct com o NavMesh em NativeArrays. |
| `Runtime/Data/NavMeshGraphBuilder.cs` | Ponte main-thread: `NavMesh.CalculateTriangulation()` → `NavMeshGraph`, incluindo solda de vértices duplicados entre tiles. |
| `Runtime/Data/NavMeshSpatialGrid.cs` | Grid uniforme XZ pra localizar triângulo por posição. |
| `Runtime/Jobs/BuildTriangleAdjacencyJob.cs` | Descobre vizinhos de cada triângulo (1x por bake). |
| `Runtime/Jobs/BuildTriangleGridJob.cs` | Monta o `NavMeshSpatialGrid` (1x por bake). |
| `Runtime/Jobs/PathRequest.cs` | Struct de pedido de caminho. |
| `Runtime/Jobs/FindPathsBatchJob.cs` | A* + funnel, em lote, por frame. |
| `Runtime/Jobs/BuildAgentSpatialHashJob.cs` | Grid de agentes por frame (pra avoidance). |
| `Runtime/Jobs/AvoidanceAndMoveJob.cs` | Avoidance + integração de movimento + clamp na malha; branch corredor individual vs. flow field. |
| `Runtime/Jobs/ComputeFlowFieldJob.cs` | Dijkstra a partir do destino sobre todos os triângulos, pra `MoveGroupWithFlowField`. |
| `Runtime/Queries/NavMeshQueryUtil.cs` | `ClosestPointOnTriangle`, `FindNearestTriangle`, `IsAreaAllowed`. |
| `Runtime/Queries/NativeMinHeap.cs` | Heap binário usado como open list do A*. |
| `Runtime/Queries/Funnel.cs` | String-pulling (Simple Stupid Funnel Algorithm). |
| `Runtime/Components/NavMeshJobManager.cs` | Orquestrador (singleton por cena). |
| `Runtime/Components/CustomNavMeshAgent.cs` | Componente por agente. |
| `Runtime/Components/PathStatus.cs` | Enum de status de caminho. |
| `Runtime/Components/MovementFaultType.cs` | Enum de falha de movimento (NaN/perda de NavMesh) — diagnóstico, ver "Diagnóstico de travamentos silenciosos". |
