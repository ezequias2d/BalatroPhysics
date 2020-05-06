using System;
using System.Collections.Generic;
using System.Text;

namespace BalatroPhysics
{
    /// <summary>
    /// Object pool interface
    /// </summary>
    /// <typeparam name="T">Type of cache</typeparam>
    public interface IObjectPool<T>
    {
        int Count { get; }

        bool IsThreadSafe { get; }

        T Get();

        void GiveBack(T item);

        void Clear();
    }
}
