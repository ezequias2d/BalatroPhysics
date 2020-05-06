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
    public class MinkowskiSumShape : Shape
    {
        private Vector3 shifted;
        private List<Shape> shapes = new List<Shape>();

        public Vector3 Shift
        {
            get
            {
                return shifted;
            }
        }

        public MinkowskiSumShape(IEnumerable<Shape> shapes)
        {
            AddShapes(shapes);
        }

        public void AddShapes(IEnumerable<Shape> shapes)
        {
            foreach (Shape shape in shapes)
            {
                if (shape is Multishape) throw new Exception("Multishapes not supported by MinkowskiSumShape.");
                this.shapes.Add(shape);
            }

            UpdateShape();
        }

        public void AddShape(Shape shape)
        {
            if (shape is Multishape) throw new Exception("Multishapes not supported by MinkowskiSumShape.");
            shapes.Add(shape);

            UpdateShape();
        }

        public bool Remove(Shape shape)
        {
            if (shapes.Count == 1) throw new Exception("There must be at least one shape.");
            bool result = shapes.Remove(shape);
            UpdateShape();
            return result;
        }

        public override void CalculateMassInertia()
        {
            MassCenterInertia = Shape.CalculateMassInertia(this);
            shifted = -shifted;
        }

        public override Vector3 SupportMapping(Vector3 direction)
        {
            Vector3 temp1 = Vector3.Zero;

            for (int i = 0; i < shapes.Count; i++)
            {
                temp1 += shapes[i].SupportMapping(direction);
            }

            return temp1 + shifted;
        }

    }
}
