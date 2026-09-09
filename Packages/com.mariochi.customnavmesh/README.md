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

### Rede de segurança: `Stale Detection Interval`

A chamada explícita acima continua sendo a via recomendada (reage no mesmo frame). Mas
antes desta seção existir, se o jogo **esquecesse** de chamar `NotifyNavMeshChanged()` em
algum lugar (um sistema de terceiros mexendo no NavMesh sem saber que este pacote existe,
por exemplo), o grafo ficava desatualizado **pra sempre**, sem nenhum log ou aviso — o
sistema não tinha como saber que estava obsoleto, só quem escreveu aquele código sabia.

`NavMeshJobManager` agora tem um `Stale Detection Interval` (default 2s, inspector) que
faz um polling periódico barato: a cada esse intervalo, compara uma assinatura leve do
estado atual contra a do grafo em uso — se diferir, chama `NotifyNavMeshChanged()`
sozinho e loga um `Debug.Log` avisando que isso aconteceu. Isso **não** é a mesma ideia
(falha) do `NavMesh.onPreUpdate` mencionado acima: aquele dispara a cada tick do
subsistema de navegação mesmo sem nada ter mudado; este polling só faz o trabalho de
verificação no ritmo do intervalo configurado (não a cada frame) e só age quando a
assinatura realmente difere, então não corre o risco de reempurrar um debounce que nunca
fecha. Configure `Stale Detection Interval <= 0` pra desligar esse polling e voltar ao
comportamento 100% manual (custo zero fora do intervalo, mas sem essa rede de segurança).

**A assinatura cobre DUAS fontes de mudança independentes**: a triangulação em si (nº de
vértices/índices, soma bruta de coordenadas — via `NavMesh.CalculateTriangulation()`) E
os `NavMeshLink` ativos da cena (contagem + soma de posição das duas pontas de cada um).
As duas são necessárias porque um `NavMeshLink` sendo adicionado, movido, ativado/
desativado ou removido em runtime **não muda a triangulação** — `NavMesh.
CalculateTriangulation()` simplesmente não sabe que links existem (ver seção
"NavMeshLink" acima). Antes de cobrir os dois, `StalenessWatcher` era estruturalmente
CEGO a qualquer mudança de link: mexer só num `NavMeshLink` nunca disparava rebuild
automático, exigindo `NotifyNavMeshChanged()` manual sempre nesse caso específico — e
isso não estava documentado em lugar nenhum, então era fácil não perceber a lacuna até um
link parar de funcionar em runtime sem nenhum aviso.

## Obstáculos dinâmicos (avoidance vs. objetos que não são agentes)

O ORCA descrito acima só enxerga outros `CustomNavMeshAgent` — uma caixa física
empurrável, uma porta fechando devagar, uma pedra rolando, uma plataforma móvel não
existiam pro avoidance, mesmo que fisicamente estivessem bem na frente de um agente.
`CustomNavMeshObstacle` ([Runtime/Components/CustomNavMeshObstacle.cs](Runtime/Components/CustomNavMeshObstacle.cs))
cobre isso: adicione o componente em qualquer GameObject que se mova por conta própria
(física, animação, outro script — qualquer coisa que não seja um agente deste pacote) e
ajuste o `Radius`. Todo frame, `NavMeshJobManager` lê `transform.position` de cada
obstáculo registrado (síncrono, main thread — aceitável porque o nº esperado de
obstáculos dinâmicos é bem menor que o de agentes) e estima a velocidade por diferença de
posição frame a frame (não há outra forma de saber a velocidade de algo que não passa por
uma API explícita). Cada obstáculo vira uma restrição ORCA extra no espaço de velocidades
de todo agente próximo, com uma diferença importante em relação a outro agente: a
responsabilidade pelo desvio é **100% do agente** (não-recíproca) — o obstáculo não sabe
nada de ORCA e não desvia de ninguém, exatamente como a RVO2/ORCA original trata
obstáculos estáticos/não-recíprocos.

- **`Obstacle Capacity`** (inspector do `NavMeshJobManager`, default 32) — pool separado
  da capacidade de agentes, mesmo padrão de buffer fixo alocado no `Awake`.
- Por baixo dos panos, obstáculos entram no MESMO grid espacial dos agentes
  (`NativeParallelMultiHashMap` compartilhado), codificados como índice **negativo**
  (`-(obstacleIndex+1)`) — evita alocar um segundo hashmap só pra isso.
- **Diferente de `NavMeshObstacle` nativo** (que é sobre TOPOLOGIA — carving, precisa de
  `RebuildGraph()`/`NotifyNavMeshChanged()` pra ter efeito, serve pra coisas
  praticamente estáticas): `CustomNavMeshObstacle` é sobre avoidance LOCAL em tempo
  real, recalculado todo frame a partir da posição atual, sem tocar o grafo/topologia.
  Os dois são complementares — um portão que abre/fecha permanentemente é caso pro
  `NavMeshObstacle` nativo + `NotifyNavMeshChanged()`; uma pedra que rola pela cena
  inteira é caso pro `CustomNavMeshObstacle`.

## NavMeshLink (off-mesh links — pulos, plataformas, zip-lines)

`NavMesh.CalculateTriangulation()` só devolve a superfície triangulada — links (saltos,
`NavMeshLink` do Unity) não entram nisso. `NavMeshGraphBuilder.BuildLinks`
([Runtime/Data/NavMeshGraphBuilder.cs](Runtime/Data/NavMeshGraphBuilder.cs)) fecha essa
lacuna: a cada `RebuildGraph()`, escaneia os componentes `NavMeshLink` ativos da cena
(`Object.FindObjectsOfType`, não tem outro jeito — não são parte da triangulação) e monta
arestas "virtuais" extras entre o triângulo mais próximo de cada ponta do link, com custo
= distância × custo de área (`costModifier >= 0` do link vira custo explícito, igual a
API nativa; `costModifier < 0`, o default, usa o custo da área). Basta ter o componente
`NavMeshLink` na cena, posicionado como de costume — nenhuma configuração extra deste
pacote é necessária.

`NavMeshLink.width` também é lido e usado: sem isso, todo agente cruzando o mesmo link ao
mesmo tempo mirava exatamente a mesma linha central (ponto de partida/chegada idênticos
pra todo mundo), então um grupo atravessando uma ponte/zip-line larga junto ficava
visualmente sobreposto numa fila de largura zero em vez de espalhado pela largura real.
Cada agente recebe um deslocamento lateral determinístico (sequência da razão áurea sobre
`AgentIndex` — bem distribuído, sem precisar coordenar entre agentes) dentro de `±width/2`,
reprojetado no triângulo real mais próximo pra nunca "pousar" fora da malha mesmo com um
`width` generoso.

O A* de `FindPathsBatchJob` considera essas arestas ALÉM da adjacência normal entre
triângulos, então um caminho pode atravessar um ou mais links no meio do trajeto. O
corredor final (`Funnel`/`BuildCorridorWithLinks`) funila normalmente dentro de cada
trecho contínuo de malha e insere, na fronteira de cada link, dois pontos exatos (início
e fim do link) sem funil — é um salto reto, não uma curva pela malha. `AvoidanceAndMoveJob`
sabe quando um agente está atravessando esse trecho (`CorridorIsLinkArrival`, marcado por
waypoint) e **desliga o clamp de superfície** só nesse segmento — sem isso, o agente seria
puxado de volta pro triângulo real mais próximo (tipicamente bem abaixo, do outro lado de
um vão) em vez de atravessar o espaço vazio em linha reta até o ponto de pouso.

**Limitações desta implementação** (funcional, mas mais simples que o suporte nativo):
- **Sem arco/curva** — a travessia é sempre uma reta 3D entre os dois pontos do link, na
  `MaxSpeed` normal do agente. Não há noção de "pular" com trajetória parabólica.
- **Avoidance entre agentes continua ativo durante o salto** — dois agentes cruzando o
  mesmo link ao mesmo tempo ainda tentam se desviar um do outro via ORCA, o que não faz
  muito sentido fisicamente no ar; simplificação deliberada (evitar isso exigiria mais um
  flag e mais complexidade por um caso de borda raro).
- **Flow field (`MoveGroupWithFlowField`) NÃO usa links** — o Dijkstra de
  `ComputeFlowFieldJob` só considera a adjacência normal entre triângulos. Um grupo que
  precisa atravessar um link deve ser individual (`SetDestination`) nesse trecho, ou
  promovido do flow field pro pipeline individual perto do link (mesmo mecanismo de
  `Flow Field Arrive Distance` já usado pra chegada).
- **`GetIsOnNavMesh()` continua `true` durante o salto** (o `CurrentTriangle` fica
  congelado no triângulo de partida) — é uma simplificação deliberada: o agente está
  numa travessia sancionada, não "perdido" (ver `MovementFaultType.LostNavMesh`, que é
  sobre outra coisa).

## Debug visual

O `NavMeshJobManager` tem cinco toggles de Gizmo (Scene view, em Play mode):

- **`Draw Status Gizmos`** — uma esfera colorida sobre cada agente com o `PathStatus`
  atual: cinza = nunca pediu path, verde = `Success`, amarelo = `PartialCorridor`,
  vermelho = `NoPath` (início/fim válidos mas sem rota entre eles, e nem um corredor de
  melhor esforço foi possível), laranja = `BestEffort` (destino inalcançável, mas o
  agente foi levado até o ponto mais perto possível dentro da própria "ilha" de NavMesh —
  ver "Destino inalcançável" abaixo), magenta = `Invalid` (início ou fim caiu fora da área
  coberta pelo NavMesh), ciano = `FlowField` (seguindo campo de grupo). **É o primeiro
  lugar pra olhar se um agente não se move** — se ele estiver vermelho ou magenta, o
  problema é o destino pedido, não o pipeline de movimento.
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

## Destino inalcançável (ilhas de NavMesh desconectadas)

Se início e fim de um `SetDestination` são válidos (ambos em cima do NavMesh) mas não há
caminho conectando os dois — ilhas diferentes de um NavMesh com múltiplos tiles/pedaços,
um destino do outro lado de um buraco ou parede sem ligação — o `NavMeshAgent` nativo do
Unity tem `pathPartialResult`/`NavMeshPathStatus.PathPartial`: em vez de deixar o agente
parado, ele monta um caminho até o ponto alcançável mais próximo do destino real.

`FindPathsBatchJob` agora faz o equivalente: durante o A*, rastreia qual triângulo
FECHADO fica heuristicamente mais perto do destino (mesmo sem nunca alcançá-lo). Se a
busca esgotar sem achar `endTri`, monta um corredor até esse triângulo em vez de devolver
corredor vazio — o agente anda até o ponto alcançável mais próximo (ex.: encosta na
parede/buraco que separa as duas ilhas) e para lá, em vez de ficar parado na posição
inicial. `PathStatus.BestEffort` sinaliza esse caso (diferente de `PartialCorridor`, que é
sobre truncamento de buffer, não sobre o caminho em si não chegar no destino). Só cai em
`PathStatus.NoPath` puro (corredor vazio) se o triângulo de início não tiver **nenhum**
vizinho alcançável de jeito nenhum (agente genuinamente isolado num triângulo solto).

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

Se isso disparar de novo: o log te diz exatamente qual agente e qual dos tipos —
`InvalidVelocity` aponta pra matemática do avoidance/campo de fluxo produzindo `NaN`;
`LostNavMesh` aponta pra uma posição que saiu longe demais da malha num frame só (ex.:
empurrão de avoidance grande demais). Isso não deveria mais travar o agente pra sempre —
ele fica visível e, na maioria dos casos, se recupera sozinho em poucos frames; se um
agente ficar com a esfera preta por muito tempo seguido, é sinal de que está genuinamente
preso fora do NavMesh (não um solavanco de 1 frame), e vale investigar a geometria da área
onde ele está. O terceiro tipo, `ExternalPositionAdopted`, é sobre outra coisa — ver seção
abaixo.

## Movendo o Transform por fora da API

`AvoidanceAndMoveJob` escreve `transform.position` de cada agente todo frame a partir da
posição simulada internamente (`Positions`/`OutPositions`) — nada no pipeline lê o
Transform de volta pra dentro dessa simulação. Antes desta seção existir, isso significava
que qualquer código de gameplay que movesse o Transform diretamente (física de knockback
com `Rigidbody`, root motion de animação, uma cutscene reposicionando o personagem) era
**silenciosamente sobrescrito** no frame seguinte: o agente "saltava de volta" pra posição
simulada, sem log, sem aviso nenhum — a única via documentada pra empurrar um agente era
`SetVelocityOverride`, mas nada te avisava se você tivesse feito diferente por engano.

Agora `AvoidanceAndMoveJob` compara, no início de cada `Execute`, a posição atual do
Transform contra a que ele mesmo escreveu no frame anterior. Se a diferença passar de
`External Move Tolerance` (inspector do `NavMeshJobManager`, default 5cm — folga o
bastante pra não disparar por ruído de ponto flutuante da própria escrita do Job), a
posição externa é **adotada** como novo ponto de partida da simulação deste frame
(reclampada no NavMesh, igual um `Warp()` implícito), em vez de descartada. Se a posição
externa estiver longe demais do NavMesh pra reclampar (teleporte pro vazio, geometria
destruída embaixo do agente), a adoção é ignorada e o fallback normal de `ClampToNavMesh`
entra em ação (ver seção acima).

Isso é sinalizado via `MovementFaultType.ExternalPositionAdopted` — logado como
`Debug.Log` (não `LogWarning`: normalmente é esperado, ex. um knockback de combate) uma
vez por ocorrência contígua. Se você **não** esperava ver esse log, é sinal de que algum
script está escrevendo `transform.position` do agente diretamente em vez de usar
`SetVelocityOverride`/`Warp()` — vale caçar esse código, porque ele está brigando com o
Job por controle da posição a cada frame.

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

## Múltiplas instâncias / multi-cena

`NavMeshJobManager.Instance` é um singleton estático — pensado pro caso comum (uma cena,
um manager, zero configuração). Antes, uma SEGUNDA instância na cena (ou numa cena
carregada aditivamente) era **destruída** no `Awake()`: qualquer setup multi-cena com
várias áreas de navmesh logicamente separadas (ex.: instância de masmorra por jogador,
cada uma com seu próprio `NavMeshJobManager`/`NavMeshSurface`) quebrava silenciosamente —
só a primeira instância sobrevivia, e todo `CustomNavMeshAgent`/`CustomNavMeshObstacle`
de QUALQUER cena carregada se registrava nela, mesmo pertencendo a uma área com
coordenadas/grafo completamente diferentes (o agente simplesmente nunca achava um
triângulo próximo e ficava com `MovementFaultType.LostNavMesh`, sem nenhuma pista óbvia
de que a causa era "registrado no manager errado").

Agora cada instância de `NavMeshJobManager` sobrevive e funciona de forma
**independente** (seu próprio grafo, seus próprios agentes/obstáculos, seu próprio
pipeline de Jobs por frame) — só a **primeira** continua virando o `Instance` estático
(o alvo default). Pra apontar um agente/obstáculo pra uma instância ESPECÍFICA em vez do default,
preencha o campo `Manager` no Inspector do `CustomNavMeshAgent`/`CustomNavMeshObstacle`
(arraste a referência do `NavMeshJobManager` da cena/área correta) antes dele ser
habilitado — `CustomNavMeshAgent.Manager`/`CustomNavMeshObstacle.Manager` (propriedade
pública, só leitura) resolve pro campo explícito se setado, senão cai pro `Instance`
padrão. Cada `NavMeshJobManager` de uma cena carregada aditivamente é independente —
agentes dessa cena devem apontar pra ELE, não confiar no default (que aponta pro manager
da primeira cena carregada).

**O que isso NÃO resolve** (limitação de fundo, não um bug): `NavMeshGraphBuilder.
BuildLinks` (`Object.FindObjectsOfType<NavMeshLink>()`) e o `NavMesh` do Unity em si
continuam sendo GLOBAIS por natureza (uma triangulação combinando todos os tiles/cenas
carregadas, ver seção "NavMeshLink" acima) — múltiplas instâncias de
`NavMeshJobManager` cada uma constrói o SEU PRÓPRIO grafo a partir dessa MESMA
triangulação global, e cada uma coleta `NavMeshLink` de QUALQUER cena carregada (a menos
que `Restrict NavMeshLinks To Own Scene` esteja ligado). Multi-manager de verdade serve
pra separar POPULAÇÕES de agentes/obstáculos e pipelines de Job independentes — não cria
áreas de NavMesh fisicamente isoladas (isso já é resolvido, ou não, pelo próprio bake do
Unity).

## Determinismo (replay/rollback)

O A* de `FindPathsBatchJob` já é determinístico por natureza: cada `PathRequest` é
resolvido isoladamente (heap/gScore/cameFrom são locais àquela iteração, `Allocator.Temp`,
sem estado compartilhado entre requests do mesmo batch), então o mesmo par início/fim sobre
o mesmo grafo sempre produz o mesmo caminho, não importa em que ordem os requests do frame
rodam nos worker threads.

O **ORCA** (`AvoidanceAndMoveJob.ComputeOrcaVelocity`) já teve um problema real aqui,
corrigido: `LinearProgram1/2/3` são um método de relaxação **sequencial** — a ordem em
que as restrições (retas) entram na lista afeta qual ponto exato é escolhido quando o
conjunto de restrições tem mais de uma solução na fronteira (comportamento normal do
algoritmo, não um bug em si — a RVO2 original tem a mesma característica). O problema era
de onde essa ordem vinha: as retas eram construídas na ordem em que
`NativeParallelMultiHashMap.TryGetFirstValue`/`TryGetNextValue` devolvia os vizinhos de
cada célula — e essa ordem reflete a ordem de **inserção paralela** feita por
`BuildAgentSpatialHashJob`/`BuildObstacleSpatialHashJob` (`IJobParallelFor`, múltiplos
worker threads escrevendo no mesmo hashmap), que não é garantida estável entre execuções
idênticas (depende de qual thread chegou primeiro em cada bucket, não do conteúdo). Ou
seja: o mesmo estado exato de posições/velocidades podia produzir uma velocidade final
levemente diferente (bit a bit) dependendo de como o agendador de Jobs distribuiu o
trabalho naquele frame — inofensivo pra um jogo comum, mas quebra replay/rollback
determinístico (netcode de lockstep, replays gravados por input).

Correção: `ComputeOrcaVelocity` agora coleta os vizinhos candidatos (índices brutos, sem
montar reta nenhuma ainda) e os **ordena** (`NativeList<int>.AsArray().Sort()`) antes de
construir qualquer `OrcaLine` — a ordem final passa a depender só do CONJUNTO de vizinhos
(sempre o mesmo pro mesmo estado de posições), nunca de em que ordem o hashmap os
devolveu. Custo extra: um `Sort()` sobre uma lista tipicamente pequena (vizinhos dentro da
janela 3x3 de células), desprezível comparado ao resto do cálculo.

**Isso cobre determinismo "mesma build, mesma máquina, replay do mesmo estado"** — não é
uma garantia de bit-exatidão **entre plataformas diferentes** (isso dependeria de Burst
gerar instruções SIMD idênticas em CPUs/SOs diferentes, fora do escopo desta correção).

## Mapas grandes (ex.: 1000x1000)

O A* de `FindPathsBatchJob` **não tem raio de busca**: ele expande o grafo de triângulos
até achar o triângulo de destino (ou esgotar o grafo), então pedir um caminho de um canto
ao outro de um mapa 1000x1000 já funciona sem nenhuma configuração especial. O que precisa
de calibração pra esse tamanho de mapa é o **broad-phase**, não o pathfinding em si:

- **`Triangle Grid Cell Size`** (no `NavMeshJobManager`) — tamanho de célula do grid usado
  por `NavMeshQueryUtil.FindNearestTriangle` pra achar rápido "em qual triângulo está essa
  posição" (usado no início/fim de todo path request e no clamp de superfície por frame).
  **Calibrado automaticamente por padrão** (`Auto Triangle Grid Cell Size`, ligado): a cada
  `RebuildGraph()`, `NavMeshSpatialGrid.EstimateAverageEdgeLength` mede o comprimento médio
  de aresta dos triângulos DESTE grafo e aplica `Auto Cell Size Multiplier` (default 3) em
  cima — a mesma regra prática de "2-4× o tamanho médio de aresta" que antes precisava ser
  calibrada manualmente por cena (olhando o `Debug.Log` e ajustando na mão), agora recalculada
  sozinha, e que se adapta automaticamente tanto a um plano aberto de 1000x1000 com poucas
  centenas de triângulos grandes (grid grosso) quanto a um terreno denso com dezenas de
  milhares de triângulos pequenos (grid fino) — sem precisar saber de antemão qual é o caso.
  Reage sozinha a rebakes que mudam a densidade de triângulos. Desligue `Auto Triangle Grid
  Cell Size` e ajuste `Triangle Grid Cell Size` manualmente só se o seu navmesh tiver
  triângulos de tamanho muito desigual (uma média simples não representa bem essa
  distribuição) ou se quiser controle fino por outro motivo. De qualquer forma, o
  `Debug.Log` que o `NavMeshJobManager` imprime em todo `RebuildGraph()` mostra o nº de
  triângulos, o tamanho de célula EFETIVO (auto ou manual) e o grid resultante — dá pra
  ajustar de olho nesse número (ou ler `NavMeshJobManager.Instance.TriangleCount` em runtime).
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
- ~~**Solda de vértice falhava de forma imprevisível em fronteira de célula de
  quantização.**~~ Resolvido — `WeldVertices` usava a célula de quantização
  (`arredondar(posição / epsilon)`) como chave de hash direta, sem checar as células
  vizinhas. Dois vértices genuinamente a MENOS de `Vertex Weld Epsilon` um do outro podiam
  cair em células diferentes se estivessem em lados opostos de uma fronteira (ex.: epsilon
  0.01, vértices em x=0.0049 e x=0.0051 — diferença real de 0.0002, mas arredondam pra
  células adjacentes) — a solda falhava sem nenhum padrão previsível, dependendo da posição
  sub-milimétrica exata de cada par. Isso não era exclusivo de costura de tile: qualquer
  junção entre pedaços de malha diferentes (o canto onde múltiplas peças de parede se
  encontram, por exemplo) podia sofrer do mesmo jeito, produzindo `PathStatus.NoPath` num
  ponto específico do mapa mesmo com `Vertex Weld Epsilon` "grande o suficiente" e outras
  costuras próximas soldando normalmente. Agora `WeldVertices` também checa as 26 células
  vizinhas (janela 3x3x3) antes de decidir que um vértice é novo, com uma checagem de
  distância real (não só a célula) antes de fundir — elimina esse falso-negativo
  independente de onde a coincidência cai em relação à grade.

## Limitações conhecidas / pontos de atenção

- ~~**`NavMeshLink.width` ignorado — agentes simultâneos no mesmo link miravam a
  mesma linha central.**~~ Resolvido — ver seção "NavMeshLink" acima. Cada agente agora
  recebe um deslocamento lateral determinístico (sequência da razão áurea sobre
  `AgentIndex`) dentro de `±width/2`, reprojetado no triângulo real mais próximo (nunca
  "pousa" fora da malha, mesmo com `width` generoso — cai de volta pro ponto central sem
  deslocamento se a busca não achar nada perto do ponto deslocado).
- ~~**`Radius` do agente podia exceder `NeighborQueryRadius`/`NeighborCellSize` sem
  aviso nenhum — "parede invisível" de detecção.**~~ Resolvido —
  `AvoidanceAndMoveJob.ComputeOrcaVelocity` agora escala o raio de detecção efetivo pra
  cima automaticamente (nunca pra baixo) com base no `Radius` do próprio agente
  (`max(NeighborQueryRadius, Radius × 2)`), e a janela de busca no grid espacial escala
  junto (antes era uma janela 3x3 fixa, incapaz de enxergar além de ~1.5×`Neighbor Cell
  Size` independente do raio configurado). Agentes de raio normal não são afetados (o
  mínimo fica bem abaixo do `Neighbor Query Radius` default); só o caso patológico
  (agentes com `Radius` bem maior que o normal, ex.: chefes/unidades grandes) ganha
  detecção automaticamente correta, sem precisar recalibrar `Neighbor Query Radius`/
  `Neighbor Cell Size` manualmente por tipo de agente.
- ~~**`StalenessWatcher` estruturalmente cego a mudanças de `NavMeshLink`.**~~
  Resolvido — ver "Rede de segurança: Stale Detection Interval" acima. A assinatura
  usada pelo polling automático só cobria a triangulação (`NavMesh.
  CalculateTriangulation()`), que não inclui `NavMeshLink` nenhum — mexer só num link em
  runtime nunca disparava rebuild automático. `NavMeshSignature` agora também soma
  contagem + posição dos `NavMeshLink` ativos da cena, então as duas fontes de mudança
  são detectadas.
- ~~**Segunda instância de `NavMeshJobManager` era destruída — quebra silenciosa em
  multi-cena aditiva.**~~ Melhorado — ver "Múltiplas instâncias / multi-cena" acima.
  `Awake()` não destrói mais duplicatas: cada instância sobrevive e funciona de forma
  independente, e um novo campo `Manager` em `CustomNavMeshAgent`/`CustomNavMeshObstacle`
  permite apontar explicitamente pra uma instância específica em vez do `Instance`
  padrão (a primeira que rodou `Awake()`). **Não resolve o problema de fundo**: o
  `NavMesh` do Unity e `NavMeshLink` continuam sendo globais por natureza — múltiplas
  instâncias de `NavMeshJobManager` separam populações de agentes/pipelines de Job, não
  criam áreas de NavMesh fisicamente isoladas.
- ~~**Overflow silencioso de `int` na chave da hash espacial em mundos abertos
  grandes.**~~ Resolvido — `NavMeshSpatialGrid.CellKey`/`AvoidanceAndMoveJob.HashCellKey`
  (usadas pra indexar `NativeParallelMultiHashMap<long, int>`, tanto no grid de triângulos
  quanto no de agentes/obstáculos) eram um hash em aritmética `int` de 32 bits
  (`cell.x * 92821 + cell.y * 68917`). Com o `Triangle Grid Cell Size`/`Neighbor Cell
  Size` default (4) e coordenadas de mundo na casa de ~25.000+ unidades, esse produto
  estourava `int.MaxValue` — Burst não lança exceção por overflow, então isso dava
  wraparound SILENCIOSO, causando colisões de célula espúrias (candidatos falsos ou
  vizinhos reais perdidos) que degradavam a qualidade de `FindNearestTriangle`/avoidance
  sem nenhum sintoma óbvio, especificamente em mundos abertos grandes. Trocado por uma
  chave `long` que EMPACOTA (não faz hash de) `cell.x` nos 32 bits altos e `cell.y` nos
  32 bits baixos — uma bijeção exata, sem colisão possível pra QUALQUER `int2`, eliminando
  a classe inteira do problema (não é "um hash melhor com limite mais alto", é ter bits
  suficientes pra nunca precisar comprimir as duas coordenadas em conflito). Os dois
  `NativeParallelMultiHashMap` afetados (`NavMeshSpatialGrid.CellToTriangle` e
  `NavMeshJobManager.agentSpatialHash`) mudaram de `<int, int>` pra `<long, int>` — mudança
  mecânica em cascata por `BuildTriangleGridJob`, `NavMeshQueryUtil.FindNearestTriangle`,
  `BuildAgentSpatialHashJob`/`BuildObstacleSpatialHashJob` e `AvoidanceAndMoveJob`, sem
  mudar nenhuma lógica de busca/avoidance em si.
- ~~**`WaypointReachDistance` 0/negativo travava o agente no mesmo waypoint pra
  sempre.**~~ Resolvido — `ComputeCorridorPrefVel` só avança o cursor com
  `distance(pos, waypoint) &lt; WaypointReachDistance`; com 0 (ou negativo), essa
  comparação estrita nunca é satisfeita por ponto flutuante (`HasReachedEnd` também nunca
  vira `true`, pelo mesmo motivo). Agora clampado a um piso positivo
  (`CustomNavMeshAgent.MinWaypointReachDistance`, 1cm) em TODOS os pontos de entrada: o
  setter `CustomNavMeshAgent.WaypointReachDistance`, `OnValidate()` (pega valor digitado
  direto no Inspector), `NavMeshJobManager.SetWaypointReachDistance`, a leitura inicial em
  `RegisterAgent` (incluindo o fallback pro `Default Waypoint Reach Distance` do
  manager), e defensivamente mais uma vez dentro do próprio `ComputeCorridorPrefVel`.
- ~~**Funil sem nenhuma margem pro raio do agente — corredor colado exatamente na
  aresta/vértice, agente largo podia clipar visualmente o canto de uma parede.**~~
  Resolvido — `Funnel.BuildCorridor` agora aceita um `radius` opcional (default 0 =
  comportamento de antes) que encolhe cada portal INTERNO (aresta compartilhada entre
  dois triângulos do corredor — não os portais degenerados start/end, que são a posição
  real do agente/destino, não uma parede) simetricamente a partir das duas pontas antes
  de rodar o funil. É a técnica padrão pra dar folga de raio a um corredor de
  string-pulling: o funil em si só sabe seguir os portais que recebe, então encolher o
  portal já embute a margem sem precisar mudar a lógica do algoritmo. Se um portal for
  mais curto que 2×`Radius`, colapsa no meio exato em vez de deixar as pontas se
  cruzarem (o agente é forçado a espremer pelo centro de uma passagem mais estreita que
  seu próprio diâmetro — fisicamente correto, não um bug: uma passagem legitimamente
  estreita demais pro agente não tem solução melhor só ajustando o corredor). `Radius` é
  passado por `PathRequest` (lido de `NavMeshJobManager.radii[agentIdx]` a cada
  repath). **Limitação de fundo que isso NÃO resolve**: o NavMesh baked pelo Unity já tem
  uma erosão de bordas embutida, calibrada pelo `Agent Radius` usado NO BAKE (Navigation
  window / `NavMeshSurface`) — se o `Radius` de um `CustomNavMeshAgent` for MAIOR que
  esse valor de bake, o corredor ainda pode ficar mais perto da parede REAL (a geometria
  visual, não a triangulação) do que o `Radius` sugere, porque a malha em si já foi
  erodida por um raio menor. Esse fix garante folga em relação à triangulação que existe,
  não em relação à geometria visual original — para cobertura completa, o `Agent Radius`
  do bake deveria ser >= o maior `Radius` entre os `CustomNavMeshAgent` que vão usar aquele
  NavMesh.
- ~~**`FindObjectsOfType<NavMeshLink>()` sempre escaneava a cena/processo inteiro.**~~
  Melhorado (parcialmente — trade-off documentado) — o `NavMesh` do Unity já é global por
  natureza (uma triangulação combinando todos os tiles/cenas carregadas), então isso não é
  uma falha de escopo em si. Onde importa: multi-cena aditiva com áreas logicamente
  separadas (ex.: instância de masmorra por jogador), já que só sobrevive UM
  `NavMeshJobManager` por processo (singleton) — ele coletaria `NavMeshLink` de TODAS as
  cenas carregadas, mesmo as "de outra instância". Novo toggle
  `Restrict NavMeshLinks To Own Scene` (desligado por padrão, preserva o comportamento de
  sempre) restringe a coleta só a componentes na MESMA cena do manager quando ligado. Não
  resolve o problema de fundo (um manager por cena de verdade exigiria repensar o
  singleton) — só dá controle pra quem precisa desse caso específico.
- ~~**`ClampToNavMesh` podia, em tese, "atravessar" uma parede fina no fallback de
  busca irrestrita.**~~ Melhorado — o fallback final (`NavMeshQueryUtil.FindNearestTriangle`
  sobre o grid espacial inteiro) aceita o triângulo mais próximo em distância EUCLIDIANA,
  sem considerar se existe caminho real até lá; com paredes finas + velocidade alta (ou
  empurrão de avoidance grande) num único frame, isso podia — em tese — clampar o agente
  do lado ERRADO de um obstáculo fino. Nova camada intermediária,
  `AvoidanceAndMoveJob.TestTriangleBFS`: entre o cache de 1 anel (rápido, cobre o
  movimento normal) e o fallback irrestrito (último recurso), um BFS bounded que caminha
  só por `NavNeighbors` (adjacência REAL da malha) — por definição, não existe aresta de
  adjacência atravessando uma parede/vão sem conexão, então essa camada NUNCA pode aceitar
  o lado errado. `Wall Safe Bfs Hops` (default 6, inspector do `NavMeshJobManager`)
  controla o alcance; só roda quando o cache de 1 anel falha (não é hot path todo frame). O
  fallback irrestrito final continua existindo (e continua com o risco teórico) só pro
  caso de teleporte/`Warp`/agente genuinamente perdido, onde não há alternativa melhor.
- ~~**`Radius` negativo não era clampado em lugar nenhum do pipeline.**~~ Resolvido —
  `combinedRadius` entra de forma LINEAR (não ao quadrado) na derivação geométrica da
  reta ORCA (`AvoidanceAndMoveJob.ComputeOrcaLine`); um raio negativo invertia parte
  dessa geometria, fazendo o agente "atrair" em vez de repelir um vizinho, em vez de
  simplesmente ser rejeitado. Agora clampado em TODOS os pontos de entrada: os setters
  `CustomNavMeshAgent.Radius`/`CustomNavMeshObstacle.Radius`, `OnValidate()` dos dois
  componentes (pega valor negativo digitado direto no Inspector, que não passa pelo
  setter), `NavMeshJobManager.SetRadius`/`SetObstacleRadius`, a leitura inicial em
  `RegisterAgent`/`RegisterObstacle`, e defensivamente mais uma vez dentro do próprio
  `ComputeOrcaVelocity` (`combinedRadius = math.max(0f, selfRadius) + math.max(0f, otherRadius)`)
  — mesmo que algum caminho futuro esqueça de clampar na entrada, o Job nunca usa um
  raio negativo pra montar a reta.
- ~~**NativeArrays `Allocator.Persistent` podiam vazar entre sessões de Play sem
  Domain Reload.**~~ Resolvido — na combinação rara "Reload Domain" + "Reload Scene"
  ambos desligados (Project Settings > Editor > Enter Play Mode Settings), o mesmo
  GameObject/Component sobrevive intacto de uma sessão de Play pra outra (a cena nunca é
  descartada), mas `Awake()` continua rodando de novo a cada entrada em Play — se os
  NativeArrays da sessão anterior nunca tivessem sido dispostos (`OnDestroy()` só roda
  quando o objeto é de fato destruído, o que não acontece nesse combo), alocar por cima
  de novo vazaria memória nativa a cada ciclo Play/Stop. Duas camadas de proteção agora:
  (1) `Awake()` detecta arrays "sobrando" (`positions.IsCreated`) de uma sessão anterior e
  dispõe tudo (nativo + bookkeeping gerenciado) antes de alocar de novo; (2) um hook de
  `EditorApplication.playModeStateChanged` (só em Editor, `#if UNITY_EDITOR`, zero
  footprint em builds) força esse mesmo descarte assim que o Editor começa a sair do Play
  Mode, sem depender do `MonoBehaviour.OnDestroy()` rodar. As duas camadas chamam o mesmo
  `DisposeAllPersistent()` (extraído de `OnDestroy()`), que é idempotente — rodar mais de
  uma vez em sequência não tem custo real na segunda vez.
- ~~**`Triangle Grid Cell Size` era 100% calibração manual.**~~ Resolvido — ver
  "Mapas grandes" acima. `Auto Triangle Grid Cell Size` (ligado por padrão) calcula o
  comprimento médio de aresta dos triângulos do grafo a cada `RebuildGraph()` e aplica
  `Auto Cell Size Multiplier` em cima, cobrindo automaticamente mapas de qualquer tamanho/
  densidade sem precisar de ajuste manual por cena (mas sem impedir calibração manual pra
  quem preferir — só desligar o toggle). ~~**A média usada era simples (por CONTAGEM de
  triângulo), não por área.**~~ Melhorado — `NavMeshSpatialGrid.EstimateAverageEdgeLength`
  agora pondera cada triângulo pela sua ÁREA, não conta 1 triângulo = 1 amostra: um
  corredor estreito cheio de triângulos pequenos ao lado de um salão aberto com poucos
  triângulos grandes não puxa mais a estimativa pra baixo só por ter mais triângulos —
  a estimativa reflete a escala do espaço REALMENTE coberto. Ainda uma limitação de fundo
  (um grid uniforme é sempre um compromisso único quando a escala varia muito de região
  pra região — resolver isso de verdade exigiria uma estrutura hierárquica/não-uniforme),
  mas uma estimativa mais representativa que a média simples anterior; calibração manual
  continua disponível pra casos extremos.
- ~~**ORCA não era determinístico entre execuções (dependia da ordem de inserção
  paralela no spatial hash).**~~ Resolvido — ver seção "Determinismo (replay/rollback)"
  acima. `ComputeOrcaVelocity` agora ordena os vizinhos candidatos por índice antes de
  montar qualquer restrição, em vez de usar a ordem (não-determinística entre execuções)
  em que o hashmap os devolve.
- ~~**Triângulo degenerado podia produzir `NaN`/`Infinity` em `ClosestPointOnTriangle`.**~~
  Resolvido — a divisão baricêntrica final (`1f / (va + vb + vc)`) não tinha nenhuma
  proteção contra área ~0 (triângulo quase colinear ou com vértices coincidentes no NavMesh
  baked). Antes disso, isso só "não quebrava" a busca de triângulo mais próximo
  (`FindNearestTriangle`) por ACIDENTE — `NaN` comparado com `<` sempre dá falso, então um
  candidato `NaN` nunca "vencia" a comparação —, mas um triângulo degenerado testado
  ISOLADO (ex.: `TestTriangleAndNeighbors`, que testa só o triângulo em cache sem competir
  contra outro candidato) podia mesmo assim propagar `NaN` pro resto do pipeline. Agora
  `ClosestPointOnTriangle` detecta a área ~0 explicitamente e cai pro vértice mais próximo
  entre os 3 em vez de arriscar a divisão.
- ~~**Funil (`Funnel.TriArea2D`) comparava área com sinal contra zero exato, sem
  epsilon.**~~ Resolvido — em portais quase colineares (comuns em navmeshes com
  triângulos finos/alongados vindos de bake grosseiro), ruído de ponto flutuante
  sub-milimétrico na posição dos vértices podia fazer o sinal da área oscilar entre
  positivo/negativo, e o funil alternava de forma instável entre "apertar o lado" e
  "resetar o apex" nesses portais. As quatro comparações centrais agora usam uma
  tolerância pequena (`Funnel.AreaEpsilon`, 1e-4) em vez de zero exato.
- ~~**Transform sobrescrito silenciosamente se movido por fora da API.**~~ Resolvido —
  `AvoidanceAndMoveJob` escreve `transform.position` todo frame a partir da posição
  simulada internamente, e nada lia o Transform de volta; qualquer física de knockback,
  root motion ou cutscene mexendo direto no Transform era descartada no frame seguinte
  sem log nenhum. Agora uma divergência acima de `External Move Tolerance` é detectada e
  ADOTADA como novo ponto de partida da simulação (reclampada no NavMesh), sinalizada via
  `MovementFaultType.ExternalPositionAdopted` — ver seção "Movendo o Transform por fora da
  API" acima.
- ~~**Destino inalcançável deixava o agente parado, sem fallback.**~~ Resolvido —
  quando início/fim eram válidos mas pertenciam a ilhas de NavMesh desconectadas,
  `FindPathsBatchJob` devolvia corredor vazio (`PathStatus.NoPath`) e o agente ficava
  parado, diferente do `pathPartialResult` do `NavMeshAgent` nativo. Agora o A* rastreia o
  triângulo fechado heuristicamente mais perto do destino e monta um corredor de melhor
  esforço até ele (`PathStatus.BestEffort`) — ver seção "Destino inalcançável" acima.
- ~~**Rebuild do grafo dependia 100% de `NotifyNavMeshChanged()` manual, sem detecção de
  staleness.**~~ Resolvido (parcialmente, por design) — se o jogo esquecesse de chamar
  `NotifyNavMeshChanged()` em algum lugar, o grafo ficava desatualizado pra sempre sem
  nenhum aviso. `Stale Detection Interval` agora faz um polling periódico barato (não por
  evento a cada tick, que era o motivo do candidato óbvio — `NavMesh.onPreUpdate` — ter
  sido descartado antes) e chama `NotifyNavMeshChanged()` sozinho se a assinatura da
  triangulação mudar — ver seção "Rede de segurança: Stale Detection Interval" acima. A
  chamada explícita continua sendo a via recomendada (reage no mesmo frame; o polling só
  pega o esquecimento, com atraso de até o intervalo configurado).
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
- ~~**Avoidance era uma aproximação (soma de empurrões + média), não ORCA/LP
  completo.**~~ Resolvido — `AvoidanceAndMoveJob` agora resolve **ORCA** de verdade
  (Van den Berg, Guy, Lin, Manocha, "Reciprocal n-Body Collision Avoidance", 2011): cada
  vizinho vira uma restrição de half-plane no espaço de velocidades 2D (XZ), derivada
  geometricamente do cone de colisão entre os dois discos (posição/velocidade/raio
  combinado/`Avoidance Time Horizon`), com responsabilidade dividida 50/50 entre os dois
  agentes (cada um roda o mesmo cálculo do seu lado — reciprocidade real, não uma
  aproximação de "quem se move na mesma direção amortece"). A velocidade final é o ponto
  DENTRO de todas essas restrições (e do círculo de `Max Speed`) mais próximo da
  velocidade preferida, resolvido por programação linear sequencial 2D
  (`LinearProgram1`/`2`/`3` em `AvoidanceAndMoveJob.cs`) — `LinearProgram3` garante que o
  sistema sempre devolve alguma velocidade mesmo se o conjunto de restrições for inviável
  (multidão muito densa/gargalo apertado), relaxando progressivamente em vez de travar.
  Isso tem garantia formal de não-colisão entre pares reciprocamente cientes dentro do
  horizonte de tempo — bem diferente da abordagem anterior (soma + média + amortecimento
  ad-hoc via `Crowd Push Damping`, removido: a divisão 50/50 embutida na construção de
  cada reta já cobre o que aquele parâmetro tentava resolver). O componente vertical (Y)
  da velocidade continua fora do ORCA (rampas/escadas não geram colisão entre agentes em
  níveis diferentes — ver `Vertical Avoidance Range`, que ainda filtra vizinhos por altura
  antes de virarem restrição).
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
- ~~**Off-mesh links não eram suportados.**~~ Resolvido (com limitações — ver seção
  "NavMeshLink" acima) — `NavMesh.CalculateTriangulation()` não devolve `NavMeshLink`
  nenhum; `NavMeshGraphBuilder.BuildLinks` agora escaneia os componentes da cena e monta
  arestas virtuais extras que o A* e o funil sabem atravessar (salto reto, sem funil,
  entre os dois pontos do link). Não suporta arco/curva, flow field, nem desliga
  avoidance entre agentes durante o salto — ver limitações detalhadas na seção acima.
- **Avoidance ainda não enxerga obstáculos não-registrados.** `CustomNavMeshObstacle`
  (ver seção "Obstáculos dinâmicos" acima) resolve o caso de objetos que se movem e
  precisam ser evitados, mas só os que o jogo registrou explicitamente com o componente —
  um Rigidbody qualquer não vira obstáculo sozinho, é preciso adicionar o componente nele.
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
  ~~**Suspeita concreta sobre `NavMeshLink`.**~~ Confirmado e corrigido — `NavMeshLink`
  (usado em `NavMeshGraphBuilder.BuildLinks`, ver seção "NavMeshLink" acima) **não** é
  `UnityEngine.AI.NavMeshLink` (não existe tal coisa no módulo built-in) — é
  `Unity.AI.Navigation.NavMeshLink`, do pacote separado `com.unity.ai.navigation` (o
  mesmo que fornece `NavMeshSurface`). Faltava `using Unity.AI.Navigation;` nos dois
  arquivos que referenciam o tipo (`NavMeshGraphBuilder.cs`, `NavMeshJobManager.cs`) e a
  referência ao assembly `Unity.AI.Navigation` no `CustomNavMesh.Runtime.asmdef` — os
  dois já corrigidos, e `com.unity.ai.navigation` (`2.0.13`, a versão verificada
  funcionando neste projeto) agora é uma dependência declarada no `package.json` do
  pacote. Isso não é mais uma limitação teórica: sem essas correções, o pacote inteiro
  não compilava em NENHUM projeto que tivesse `com.unity.ai.navigation` instalado.

## Arquivos

| Arquivo | Papel |
|---|---|
| `package.json` | Manifesto UPM (nome, versão, dependências). |
| `Runtime/CustomNavMesh.Runtime.asmdef` | Assembly Definition do pacote (referencia Burst/Collections/Mathematics). |
| `Runtime/NavMeshJobConstants.cs` | Constantes compartilhadas (`MaxCorridorPoints`, `MaxNavMeshAreas`). |
| `Runtime/Data/NavMeshGraph.cs` | Struct com o NavMesh em NativeArrays. |
| `Runtime/Data/NavMeshGraphBuilder.cs` | Ponte main-thread: `NavMesh.CalculateTriangulation()` → `NavMeshGraph`, incluindo solda de vértices duplicados entre tiles e coleta de `NavMeshLink` (`BuildLinks`). |
| `Runtime/Data/NavMeshSpatialGrid.cs` | Grid uniforme XZ pra localizar triângulo por posição. |
| `Runtime/Jobs/BuildTriangleAdjacencyJob.cs` | Descobre vizinhos de cada triângulo (1x por bake). |
| `Runtime/Jobs/BuildTriangleGridJob.cs` | Monta o `NavMeshSpatialGrid` (1x por bake). |
| `Runtime/Jobs/PathRequest.cs` | Struct de pedido de caminho. |
| `Runtime/Jobs/FindPathsBatchJob.cs` | A* (incluindo arestas de `NavMeshLink`) + funnel/`BuildCorridorWithLinks`, em lote, por frame. |
| `Runtime/Jobs/BuildAgentSpatialHashJob.cs` | Grid de agentes por frame (pra avoidance). |
| `Runtime/Jobs/BuildObstacleSpatialHashJob.cs` | Insere `CustomNavMeshObstacle` no mesmo grid dos agentes (índice negativo), por frame. |
| `Runtime/Jobs/AvoidanceAndMoveJob.cs` | ORCA (avoidance) + integração de movimento + clamp na malha (com bypass durante travessia de link); branch corredor individual vs. flow field. |
| `Runtime/Jobs/ComputeFlowFieldJob.cs` | Dijkstra a partir do destino sobre todos os triângulos, pra `MoveGroupWithFlowField`. |
| `Runtime/Queries/NavMeshQueryUtil.cs` | `ClosestPointOnTriangle`, `FindNearestTriangle`, `IsAreaAllowed`. |
| `Runtime/Queries/NativeMinHeap.cs` | Heap binário usado como open list do A*. |
| `Runtime/Queries/Funnel.cs` | String-pulling (Simple Stupid Funnel Algorithm). |
| `Runtime/Components/NavMeshJobManager.cs` | Orquestrador (singleton por cena). |
| `Runtime/Components/CustomNavMeshAgent.cs` | Componente por agente. |
| `Runtime/Components/CustomNavMeshObstacle.cs` | Componente por obstáculo dinâmico (avoidance não-recíproca, sem pathfinding). |
| `Runtime/Components/PathStatus.cs` | Enum de status de caminho. |
| `Runtime/Components/MovementFaultType.cs` | Enum de falha de movimento (NaN/perda de NavMesh/adoção de posição externa) — diagnóstico, ver "Diagnóstico de travamentos silenciosos". |
