using System;
using System.Collections.Concurrent;
using System.Threading;

namespace BalatroPhysics
{
    /// <summary>
    /// Object pool to cache objects
    /// </summary>
    /// <typeparam name="T">Type of cache</typeparam>
    public class ObjectPool<T> : IObjectPool<T>
    {
        private ConcurrentStack<T> _pool;
        private Func<T> _generator;
        private int _count;

        /// <summary>
        /// Number of objects in the pool
        /// </summary>
        public int Count { get { return _count; } }

        /// <summary>
        /// True
        /// </summary>
        public bool IsThreadSafe { get { return true; } }

        /// <summary>
        /// Create a new object pool with a delegate for T object.
        /// </summary>
        /// <param name="generator">Generator</param>
        public ObjectPool(Func<T> generator)
        {
            _pool = new ConcurrentStack<T>();
            _generator = generator;
            _count = 0;
        }

        /// <summary>
        /// Remove all objects of pool
        /// </summary>
        public void Clear()
        {
            Interlocked.Exchange(ref _count, 0);
            _pool.Clear();
        }

        /// <summary>
        /// Get a object from pool, if not exists create a new using delegate "generator".
        /// </summary>
        /// <returns> A T object </returns>
        public T Get()
        {
            if(Count > 0 && _pool.TryPop(out T result))
            {
                Interlocked.Decrement(ref _count);
                return result;
            }
            return _generator();
        }

        /// <summary>
        /// Give back a object to pool
        /// </summary>
        /// <param name="item"> Object </param>
        public void GiveBack(T item)
        {
            _pool.Push(item);
            Interlocked.Increment(ref _count);
        }
    }
}
