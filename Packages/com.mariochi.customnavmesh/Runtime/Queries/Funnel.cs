using Unity.Collections;
using Unity.Mathematics;

namespace CustomNavMesh
{
    /// <summary>
    /// "Simple Stupid Funnel Algorithm" (string-pulling): transforma um corredor de
    /// triângulos (resultado do A*) numa polilinha enxuta de waypoints, colada nos
    /// cantos dos triângulos por onde o caminho reto passaria. É o mesmo algoritmo usado
    /// pelo Detour/Recast.
    ///
    /// IMPORTANTE sobre winding: GetSharedEdge assume que os triângulos do Unity têm
    /// winding consistente (o que é verdade pra qualquer NavMesh baked pelo Unity/Recast).
    /// A convenção left/right usada abaixo foi confirmada empiricamente contra o winding
    /// real do NavMesh do Unity — antes de acertar isso o corredor saía em zigue-zague em
    /// vez de reto em espaços largos (funil "esticando" errado a cada portal).
    /// </summary>
    public static class Funnel
    {
        /// <summary>
        /// Tolerância pra tratar TriArea2D como "zero" (portal ~colinear com a aresta atual
        /// do funil) em vez de comparar com zero exato. Sem isso, ruído de ponto flutuante
        /// sub-milimétrico na posição dos vértices (comum perto de junções/costuras) podia
        /// fazer o sinal da área oscilar entre positivo/negativo de um frame de bake pro
        /// outro em portais genuinamente quase colineares (comuns em navmeshes com
        /// triângulos finos/alongados vindos de bake grosseiro) — o funil alternava de forma
        /// instável entre "apertar o lado" e "resetar o apex" nesses portais. TriArea2D
        /// devolve 2x a área com sinal (escala com o comprimento das arestas, não é uma
        /// unidade fixa), então isto é uma tolerância absoluta pequena o bastante pra só
        /// pegar ruído sub-milimétrico, não decisões geométricas reais do funil (que mudam
        /// a área por ordens de grandeza maior pra qualquer portal com angulação genuína).
        /// </summary>
        const float AreaEpsilon = 1e-4f;

        /// <summary>
        /// 'radius' (default 0 = comportamento original, sem margem nenhuma): encolhe cada
        /// portal INTERNO (aresta compartilhada entre dois triângulos do corredor, NÃO os
        /// portais degenerados start/end) pra dentro por até esse valor, simetricamente a
        /// partir das duas pontas, antes de rodar o funil sobre eles. Isso é a técnica
        /// padrão pra dar folga de raio a um corredor de string-pulling (o funil em si só
        /// sabe seguir portais — ele produz exatamente o resultado ótimo em cima de
        /// QUALQUER portal que receba, então encolher o portal já embute a folga sem
        /// precisar mudar a lógica do funil): sem isso, o corredor colava exatamente na
        /// aresta/vértice físico do triângulo, e um agente com raio &gt; 0 fazendo uma curva
        /// rente a esse ponto podia clipar visualmente o canto de uma parede mesmo com o
        /// "núcleo" da simulação tecnicamente dentro do NavMesh. Se o portal for mais curto
        /// que 2×radius, encolhe só até o meio dele (colapsa num único ponto) — o agente é
        /// forçado a espremer pelo centro de uma passagem mais estreita que seu próprio
        /// diâmetro, que é o comportamento fisicamente correto (não um bug): uma passagem
        /// legitimamente estreita demais pro agente não tem solução melhor só ajustando o
        /// corredor — o problema real ali é o NavMesh ter sido baked com um raio de erosão
        /// menor que o Radius deste agente (ver README, "Funil e margem de raio").
        /// </summary>
        public static void BuildCorridor(
            float3 start,
            float3 end,
            in NativeList<int> triPath,
            in NativeArray<int3> triangles,
            in NativeArray<float3> vertices,
            NativeList<float3> outPoints,
            float radius = 0f)
        {
            outPoints.Add(start);

            int k = triPath.Length;
            if (k <= 1)
            {
                outPoints.Add(end);
                return;
            }

            int portalCount = k + 1; // portal 0 = (start,start) .. portal k = (end,end)
            var portalLeft = new NativeArray<float3>(portalCount, Allocator.Temp);
            var portalRight = new NativeArray<float3>(portalCount, Allocator.Temp);

            portalLeft[0] = start;
            portalRight[0] = start;
            for (int i = 1; i < k; i++)
            {
                GetSharedEdge(triangles[triPath[i - 1]], triangles[triPath[i]], vertices, out float3 l, out float3 r);
                if (radius > 0f) InsetPortal(ref l, ref r, radius);
                portalLeft[i] = l;
                portalRight[i] = r;
            }
            portalLeft[k] = end;
            portalRight[k] = end;

            float3 apex = portalLeft[0];
            float3 left = portalLeft[0];
            float3 right = portalRight[0];
            int apexIndex = 0, leftIndex = 0, rightIndex = 0;

            for (int i = 1; i <= k; i++)
            {
                float3 pl = portalLeft[i];
                float3 pr = portalRight[i];

                // lado direito do funil
                if (TriArea2D(apex, right, pr) <= AreaEpsilon)
                {
                    if (ApproxEqual(apex, right) || TriArea2D(apex, left, pr) > AreaEpsilon)
                    {
                        right = pr;
                        rightIndex = i;
                    }
                    else
                    {
                        outPoints.Add(left);
                        apex = left; apexIndex = leftIndex;
                        left = apex; right = apex;
                        leftIndex = rightIndex = apexIndex;
                        i = apexIndex;
                        continue;
                    }
                }

                // lado esquerdo do funil (simétrico)
                if (TriArea2D(apex, left, pl) >= -AreaEpsilon)
                {
                    if (ApproxEqual(apex, left) || TriArea2D(apex, right, pl) < -AreaEpsilon)
                    {
                        left = pl;
                        leftIndex = i;
                    }
                    else
                    {
                        outPoints.Add(right);
                        apex = right; apexIndex = rightIndex;
                        left = apex; right = apex;
                        leftIndex = rightIndex = apexIndex;
                        i = apexIndex;
                        continue;
                    }
                }
            }

            outPoints.Add(end);

            portalLeft.Dispose();
            portalRight.Dispose();
        }

        static void GetSharedEdge(in int3 triA, in int3 triB, in NativeArray<float3> vertices, out float3 left, out float3 right)
        {
            for (int e = 0; e < 3; e++)
            {
                int a = triA[e];
                int b = triA[(e + 1) % 3];
                if (Contains(triB, a) && Contains(triB, b))
                {
                    // winding do NavMesh do Unity: invertido em relação ao que eu assumi originalmente
                    // (confirmado empiricamente — sem isso o funil produz zigue-zague em vez de reta).
                    left = vertices[a];
                    right = vertices[b];
                    return;
                }
            }

            // não deveria acontecer se triA/triB são realmente vizinhos no grafo
            right = left = vertices[triA.x];
        }

        static bool Contains(in int3 tri, int v) => tri.x == v || tri.y == v || tri.z == v;

        /// <summary>
        /// Encolhe o segmento (left, right) simetricamente a partir das duas pontas, em
        /// direção ao meio, por até 'radius' — ver comentário em BuildCorridor. Se
        /// 2×radius &gt;= comprimento do segmento, colapsa os dois pontos no meio exato em
        /// vez de deixar left/right se cruzarem (o que inverteria a ordem esquerda/direita
        /// e quebraria a convenção de winding que o resto do funil depende).
        /// </summary>
        static void InsetPortal(ref float3 left, ref float3 right, float radius)
        {
            float3 edge = right - left;
            float len = math.length(edge);
            if (len < 1e-5f) return; // portal já é ~um ponto único — nada a encolher

            float inset = math.min(radius, len * 0.5f);
            float3 dir = edge / len;
            left += dir * inset;
            right -= dir * inset;
        }

        /// <summary>Área com sinal (2x) do triângulo (a,b,c) projetado no plano XZ.</summary>
        static float TriArea2D(float3 a, float3 b, float3 c)
        {
            float ax = b.x - a.x, az = b.z - a.z;
            float bx = c.x - a.x, bz = c.z - a.z;
            return bx * az - ax * bz;
        }

        static bool ApproxEqual(float3 a, float3 b) => math.distancesq(a, b) < 1e-6f;
    }
}
