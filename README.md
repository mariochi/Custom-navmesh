# Custom-Navmesh — projeto de teste

Este é o projeto Unity usado pra desenvolver e testar o pacote **CustomNavMesh**
(pathfinding + avoidance + flow fields multithreaded via Job System/Burst, lendo o
NavMesh baked padrão do Unity).

O pacote em si — código-fonte, documentação completa de arquitetura/setup/limitações —
vive em [`Packages/com.mariochi.customnavmesh/`](Packages/com.mariochi.customnavmesh/README.md).
Esse é o entregável reutilizável; este repositório inteiro é só o ambiente de teste em
volta dele (cena de exemplo, script de spawner `Assets/Scripts/TargetSeeker.cs`, etc.).

**Pra instalar o pacote em outro projeto**, ver a seção "Instalação em outro projeto" no
[README do pacote](Packages/com.mariochi.customnavmesh/README.md) — resumo rápido:

```
https://github.com/mariochi/Custom-navmesh.git?path=Packages/com.mariochi.customnavmesh
```
(Package Manager → `+` → *Add package from git URL*)
