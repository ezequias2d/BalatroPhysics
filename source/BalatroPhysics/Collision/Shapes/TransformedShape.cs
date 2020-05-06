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

using BalatroPhysics.LinearMath;
using System.Numerics;

namespace BalatroPhysics.Collision.Shapes
{
    /// <summary>
    /// Holds a 'sub' shape and it's transformation. This TransformedShape can
    /// be added to the <see cref="CompoundShape"/>
    /// </summary>
    public struct TransformedShape
    {
        private Vector3 position;
        private Matrix4x4 orientation;
        private Matrix4x4 invOrientation;
        private JBBox boundingBox;

        /// <summary>
        /// The 'sub' shape.
        /// </summary>
        public Shape Shape { get; set; }

        /// <summary>
        /// The position of a 'sub' shape
        /// </summary>
        public Vector3 Position 
        { 
            get
            { 
                return position;
            } 
            set 
            { 
                position = value; 
                UpdateBoundingBox(); 
            }
        }

        public JBBox BoundingBox { get { return boundingBox; } }

        /// <summary>
        /// The inverse orientation of the 'sub' shape.
        /// </summary>
        public Matrix4x4 InverseOrientation { get { return invOrientation; } }

        /// <summary>
        /// The orienation of the 'sub' shape.
        /// </summary>
        public Matrix4x4 Orientation
        {
            get 
            { 
                return orientation; 
            }
            set 
            {
                orientation = value;
                invOrientation = Matrix4x4.Transpose(orientation);
                UpdateBoundingBox();
            }
        }

        public void UpdateBoundingBox()
        {
            Shape.GetBoundingBox(orientation, out boundingBox);

            boundingBox.Min += position;
            boundingBox.Max += position;
        }

        /// <summary>
        /// Creates a new instance of the TransformedShape struct.
        /// </summary>
        /// <param name="shape">The shape.</param>
        /// <param name="orientation">The orientation this shape should have.</param>
        /// <param name="position">The position this shape should have.</param>
        public TransformedShape(Shape shape, Matrix4x4 orientation, Vector3 position)
        {
            this.position = position;
            this.orientation = orientation;
            invOrientation = Matrix4x4.Transpose(orientation);
            Shape = shape;
            boundingBox = new JBBox();
            UpdateBoundingBox();
        }
    }
}
