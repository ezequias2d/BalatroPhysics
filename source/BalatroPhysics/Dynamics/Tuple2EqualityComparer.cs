using System;
using System.Collections.Generic;
using System.Text;

namespace BalatroPhysics.Dynamics
{
    public class Tuple2EqualityComparer<T> : IEqualityComparer<(T, T)>
    {
        public bool Equals((T, T) x, (T, T) y)
        {
            return (x.Item1.Equals(y.Item1) && x.Item2.Equals(y.Item2)) ||
                (x.Item1.Equals(y.Item2) && x.Item2.Equals(y.Item1));
        }

        public int GetHashCode((T, T) obj)
        {
            return obj.GetHashCode();
        }
    }
}
