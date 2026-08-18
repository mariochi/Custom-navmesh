using System;
using Unity.Collections;

namespace CustomNavMesh
{
    /// <summary>
    /// Min-heap binário simples (id, prioridade) sobre NativeList, usado como open list do A*.
    /// Alocado com Allocator.Temp dentro do Execute de cada iteração do Job de pathfinding
    /// (escopo por-thread, descartado automaticamente ao fim da iteração).
    ///
    /// Permite prioridades duplicadas para o mesmo id (o Job de pathfinding trata isso com
    /// "lazy deletion": ao dar Pop, se o nó já está fechado, ele é simplesmente ignorado).
    /// </summary>
    public struct NativeMinHeap : IDisposable
    {
        struct HeapItem
        {
            public int Id;
            public float Priority;
        }

        NativeList<HeapItem> items;

        public NativeMinHeap(int capacity, Allocator allocator)
        {
            items = new NativeList<HeapItem>(capacity, allocator);
        }

        public int Count => items.Length;

        public void Push(int id, float priority)
        {
            items.Add(new HeapItem { Id = id, Priority = priority });
            int i = items.Length - 1;
            while (i > 0)
            {
                int parent = (i - 1) / 2;
                if (items[parent].Priority <= items[i].Priority) break;
                Swap(parent, i);
                i = parent;
            }
        }

        public int Pop()
        {
            HeapItem root = items[0];
            int last = items.Length - 1;
            items[0] = items[last];
            items.RemoveAt(last);

            int i = 0;
            int n = items.Length;
            while (true)
            {
                int l = 2 * i + 1, r = 2 * i + 2, smallest = i;
                if (l < n && items[l].Priority < items[smallest].Priority) smallest = l;
                if (r < n && items[r].Priority < items[smallest].Priority) smallest = r;
                if (smallest == i) break;
                Swap(smallest, i);
                i = smallest;
            }

            return root.Id;
        }

        void Swap(int a, int b)
        {
            HeapItem tmp = items[a];
            items[a] = items[b];
            items[b] = tmp;
        }

        public void Dispose() => items.Dispose();
    }
}
