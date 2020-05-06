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
using System.Numerics;
#endregion

namespace BalatroPhysics.Collision.Shapes
{

    /// <summary>
    /// A <see cref="Shape"/> representing a cylinder.
    /// </summary>
    public class CylinderShape : Shape
    {
        private float height;
        private float radius;

        /// <summary>
        /// Sets the height of the cylinder.
        /// </summary>
        public float Height 
        { 
            get 
            {
                return height;
            }
            set 
            { 
                height = value;
                UpdateShape();
            } 
        }

        /// <summary>
        /// Sets the radius of the cylinder.
        /// </summary>
        public float Radius
        { 
            get 
            { 
                return radius;
            } 
            set 
            { 
                radius = value;
                UpdateShape(); 
            } 
        }

        /// <summary>
        /// Initializes a new instance of the CylinderShape class.
        /// </summary>
        /// <param name="height">The height of the cylinder.</param>
        /// <param name="radius">The radius of the cylinder.</param>
        public CylinderShape(float height, float radius)
        {
            this.height = height;
            this.radius = radius;
            UpdateShape();
        }

        /// <summary>
        /// 
        /// </summary>
        public override void CalculateMassInertia()
        {
            Mass = JMath.Pi * radius * radius * height;
            Inertia = JMath.MatrixFromM11M22M33(
                (1.0f / 4.0f) * Mass * radius * radius + (1.0f / 12.0f) * Mass * height * height,
                (1.0f / 2.0f) * Mass * radius * radius,
                (1.0f / 4.0f) * Mass * radius * radius + (1.0f / 12.0f) * Mass * height * height);
        }

        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        public override Vector3 SupportMapping(Vector3 direction)
        {
            float sigma = (float)Math.Sqrt((float)(direction.X * direction.X + direction.Z * direction.Z));

            if (sigma > 0.0f)
                return new Vector3(direction.X / sigma * radius, Math.Sign(direction.Y) * height * 0.5f, direction.Z / sigma * radius);
            else
                return new Vector3(0f, Math.Sign(direction.Y) * height * 0.5f, 0f);
        }
    }
}
