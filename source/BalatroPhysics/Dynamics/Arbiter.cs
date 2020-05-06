﻿/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
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
using System.Diagnostics;
using System.Numerics;
#endregion

namespace BalatroPhysics.Dynamics
{
    /// <summary>
    /// An arbiter holds all contact information of two bodies.
    /// The contacts are stored in the ContactList. There is a maximum
    /// of four contacts which can be added to an arbiter. The arbiter
    /// only keeps the best four contacts based on the area spanned by
    /// the contact points.
    /// </summary>
    public class Arbiter
    {
        /// <summary>
        /// The first body.
        /// </summary>
        public RigidBody Body1 { get { return body1; } }

        /// <summary>
        /// The second body.
        /// </summary>
        public RigidBody Body2 { get { return body2; } }

        /// <summary>
        /// The contact list containing all contacts of both bodies.
        /// </summary>
        public ContactList ContactList { get { return contactList; } }

        /// <summary>
        /// </summary>
        public static ResourcePool<Arbiter> Pool = new ResourcePool<Arbiter>();

        // internal values for faster access within the engine
        internal RigidBody body1, body2;
        internal ContactList contactList;

        /// <summary>
        /// </summary>
        /// <param name="body1"></param>
        /// <param name="body2"></param>
        public Arbiter(RigidBody body1, RigidBody body2)
        {
            this.contactList = new ContactList();
            this.body1 = body1;
            this.body2 = body2;
        }

        /// <summary>
        /// Initializes a new instance of the Arbiter class.
        /// </summary>
        public Arbiter()
        {
            this.contactList = new ContactList();
        }

        /// <summary>
        /// Removes all contacts from this arbiter.
        /// The world will remove the arbiter automatically next frame
        /// or add new contacts.
        /// </summary>
        public void Invalidate()
        {
            contactList.Clear();
        }

        /// <summary>
        /// Adds a contact to the arbiter (threadsafe). No more than four contacts 
        /// are stored in the contactList. When adding a new contact
        /// to the arbiter the existing are checked and the best are kept.
        /// </summary>
        /// <param name="point1">Point on body1. In world space.</param>
        /// <param name="point2">Point on body2. In world space.</param>
        /// <param name="normal">The normal pointing to body2.</param>
        /// <param name="penetration">The estimated penetration depth.</param>
        public Contact AddContact(Vector3 point1, Vector3 point2, Vector3 normal, float penetration, 
            ContactSettings contactSettings)
        {
            Vector3 relPos1 = point1 - body1.position;

            int index;

            lock (contactList)
            {
                if (this.contactList.Count == 4)
                {
                    index = SortCachedPoints(relPos1, penetration);
                    ReplaceContact(point1, point2, normal, penetration, index, contactSettings);
                    return null;
                }

                index = GetCacheEntry(relPos1, contactSettings.breakThreshold);

                if (index >= 0)
                {
                    ReplaceContact(point1, point2, normal, penetration, index, contactSettings);
                    return null;
                }
                else
                {
                    Contact contact = Contact.Pool.GetNew();
                    contact.Initialize(body1, body2, point1, point2, normal, penetration, true, contactSettings);
                    contactList.Add(contact);
                    return contact;
                }
            }
        }

        private void ReplaceContact(Vector3 point1, Vector3 point2, Vector3 n, float p, int index,
            ContactSettings contactSettings)
        {
            Contact contact = contactList[index];

            Debug.Assert(body1 == contact.body1, "Body1 and Body2 not consistent.");

            contact.Initialize(body1, body2, point1, point2, n, p, false, contactSettings);

        }

        private int GetCacheEntry(Vector3 realRelPos1, float contactBreakThreshold)
        {
            float shortestDist = contactBreakThreshold * contactBreakThreshold;
            int size = contactList.Count;
            int nearestPoint = -1;
            for (int i = 0; i < size; i++)
            {
                Vector3 diffA = contactList[i].relativePos1 - realRelPos1;
                float distToManiPoint = diffA.LengthSquared();
                if (distToManiPoint < shortestDist)
                {
                    shortestDist = distToManiPoint;
                    nearestPoint = i;
                }
            }
            return nearestPoint;
        }

        // sort cached points so most isolated points come first
        private int SortCachedPoints(Vector3 realRelPos1, float pen)
        {
            //calculate 4 possible cases areas, and take biggest area
            //also need to keep 'deepest'

            int maxPenetrationIndex = -1;
            float maxPenetration = pen;
            for (int i = 0; i < 4; i++)
            {
                if (contactList[i].penetration > maxPenetration)
                {
                    maxPenetrationIndex = i;
                    maxPenetration = contactList[i].penetration;
                }
            }

            float res0 = 0, res1 = 0, res2 = 0, res3 = 0;
            if (maxPenetrationIndex != 0)
            {
                Vector3 a0 = realRelPos1 - contactList[1].relativePos1;
                Vector3 b0 = contactList[3].relativePos1 - contactList[2].relativePos1;
                Vector3 cross = Vector3.Cross(a0, b0);
                res0 = cross.LengthSquared();
            }
            if (maxPenetrationIndex != 1)
            {
                Vector3 a0 = realRelPos1 - contactList[0].relativePos1;
                Vector3 b0 = contactList[3].relativePos1 - contactList[2].relativePos1;
                Vector3 cross = Vector3.Cross(a0, b0);
                res1 = cross.LengthSquared();
            }

            if (maxPenetrationIndex != 2)
            {
                Vector3 a0 = realRelPos1 - contactList[0].relativePos1;
                Vector3 b0 = contactList[3].relativePos1 - contactList[1].relativePos1;
                Vector3 cross = Vector3.Cross(a0, b0);
                res2 = cross.LengthSquared();
            }

            if (maxPenetrationIndex != 3)
            {
                Vector3 a0 = realRelPos1 - contactList[0].relativePos1;
                Vector3 b0 = contactList[2].relativePos1 - contactList[1].relativePos1;
                Vector3 cross = Vector3.Cross(a0, b0);
                res3 = cross.LengthSquared();
            }

            int biggestarea = MaxAxis(res0, res1, res2, res3);
            return biggestarea;
        }

        internal static int MaxAxis(float x, float y, float z, float w)
        {
            int maxIndex = -1;
            float maxVal = float.MinValue;

            if (x > maxVal) { maxIndex = 0; maxVal = x; }
            if (y > maxVal) { maxIndex = 1; maxVal = y; }
            if (z > maxVal) { maxIndex = 2; maxVal = z; }
            if (w > maxVal) { maxIndex = 3; maxVal = w; }

            return maxIndex;
        }

    }
}
