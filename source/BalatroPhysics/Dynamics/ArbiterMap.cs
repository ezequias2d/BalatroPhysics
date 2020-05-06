/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

#region Using Statements
using System;
using System.Collections.Generic;

using BalatroPhysics.Dynamics;
using BalatroPhysics.LinearMath;
using BalatroPhysics.Collision.Shapes;
using System.Collections;
using BalatroPhysics.Collision;
#endregion

namespace BalatroPhysics.Dynamics
{

    /// <summary>
    /// The ArbiterMap is a dictionary which stores all arbiters.
    /// </summary>
    public class ArbiterMap : ICollection<Arbiter>
    {
        private Dictionary<(RigidBody, RigidBody), Arbiter> dictionary =
            new Dictionary<(RigidBody, RigidBody), Arbiter>(2048, arbiterKeyComparer);

        private (RigidBody, RigidBody) lookUpKey;
        private static Tuple2EqualityComparer<RigidBody> arbiterKeyComparer = new Tuple2EqualityComparer<RigidBody>();

        public int Count
        {
            get
            {
                return dictionary.Count;
            }
        }

        public bool IsReadOnly
        {
            get
            {
                return false;
            }
        }

        /// <summary>
        /// Initializes a new instance of the ArbiterMap class.
        /// </summary>
        public ArbiterMap()
        {
            lookUpKey = default((RigidBody, RigidBody));
        }

        /// <summary>
        /// Gets an arbiter by it's bodies. Not threadsafe.
        /// </summary>
        /// <param name="body1">The first body.</param>
        /// <param name="body2">The second body.</param>
        /// <param name="arbiter">The arbiter which was found.</param>
        /// <returns>Returns true if the arbiter could be found, otherwise false.</returns>
        public bool LookUpArbiter(RigidBody body1, RigidBody body2,out Arbiter arbiter)
        {
            lookUpKey = (body1, body2);
            return dictionary.TryGetValue(lookUpKey, out arbiter);
        }

        public void Add(Arbiter arbiter)
        {
            dictionary.Add((arbiter.body1, arbiter.body2), arbiter);
        }

        public void Clear()
        {
            dictionary.Clear();
        }

        public bool Remove(Arbiter arbiter)
        {
            lookUpKey = (arbiter.body1, arbiter.body2);
            return dictionary.Remove(lookUpKey);
        }

        /// <summary>
        /// Checks if an arbiter is within the arbiter map.
        /// </summary>
        /// <param name="body1">The first body.</param>
        /// <param name="body2">The second body.</param>
        /// <returns>Returns true if the arbiter could be found, otherwise false.</returns>
        public bool Contains(RigidBody body1, RigidBody body2)
        {
            lookUpKey = (body1, body2);
            return dictionary.ContainsKey(lookUpKey);
        }

        public bool Contains(Arbiter arbiter)
        {
            lookUpKey = (arbiter.body1, arbiter.body2);
            return dictionary.ContainsKey(lookUpKey);
        }

        public void CopyTo(Arbiter[] array, int arrayIndex)
        {
            dictionary.Values.CopyTo(array, arrayIndex);
        }

        public IEnumerator<Arbiter> GetEnumerator()
        {
            return dictionary.Values.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }

}
