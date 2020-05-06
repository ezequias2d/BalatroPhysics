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
    /// A <see cref="Shape"/> representing a compoundShape consisting
    /// of several 'sub' shapes.
    /// </summary>
    public class CompoundShape : Multishape
    {
        private TransformedShape[] _shapes;
        /// <summary>
        /// An array conaining all 'sub' shapes and their transforms.
        /// </summary>
        public TransformedShape[] Shapes { get { return _shapes; } }

        Vector3 shifted;
        public Vector3 Shift { get { return -1.0f * this.shifted; } }

        private JBBox mInternalBBox;

        /// <summary>
        /// Created a new instance of the CompountShape class.
        /// </summary>
        /// <param name="shapes">The 'sub' shapes which should be added to this 
        /// class.</param>
        public CompoundShape(List<TransformedShape> shapes)
        {
            _shapes = new TransformedShape[shapes.Count];
            shapes.CopyTo(_shapes);

            if (!TestValidity()) 
                throw new ArgumentException("Multishapes are not supported!");

            this.UpdateShape();
        }

        public CompoundShape(TransformedShape[] shapes)
        {
            _shapes = new TransformedShape[shapes.Length];
            Array.Copy(shapes, _shapes, shapes.Length);

            if (!TestValidity())
                throw new ArgumentException("Multishapes are not supported!");

            this.UpdateShape();
        }

        private CompoundShape(ref TransformedShape[] shapes)
        {
            _shapes = shapes;

            if (!TestValidity())
                throw new ArgumentException("Multishapes are not supported!");

            UpdateShape();
        }

        private bool TestValidity()
        {
            for (int i = 0; i < Shapes.Length; i++)
            {
                if (Shapes[i].Shape is Multishape) return false;
            }

            return true;
        }

        public override void MakeHull(List<Vector3> triangleList, int generationThreshold)
        {
            List<Vector3> triangles = new List<Vector3>();

            for (int i = 0; i < Shapes.Length; i++)
            {
                Shapes[i].Shape.MakeHull(triangles, 4);
                for (int e = 0; e < triangles.Count; e++)
                {
                    Vector3 pos = triangles[e];
                    JMath.Transform(pos, Shapes[i].Orientation,out pos);
                    pos += Shapes[i].Position;
                    triangleList.Add(pos);
                }
                triangles.Clear();
            }
        }

        /// <summary>
        /// Translate all subshapes in the way that the center of mass is
        /// in (0,0,0)
        /// </summary>
        private void DoShifting()
        {
            for (int i = 0; i < Shapes.Length; i++) shifted += Shapes[i].Position;
            shifted *= (1.0f / Shapes.Length);

            for (int i = 0; i < Shapes.Length; i++) Shapes[i].Position -= shifted;
        }

        public override void CalculateMassInertia()
        {
            Inertia = JMath.ZeroMatrix;
            Mass = 0.0f;

            for (int i = 0; i < Shapes.Length; i++)
            {
                Matrix4x4 currentInertia = Shapes[i].InverseOrientation * Shapes[i].Shape.Inertia * Shapes[i].Orientation;
                Vector3 p = Shapes[i].Position * -1.0f;
                float m = Shapes[i].Shape.Mass;

                currentInertia.M11 += m * (p.Y * p.Y + p.Z * p.Z);
                currentInertia.M22 += m * (p.X * p.X + p.Z * p.Z);
                currentInertia.M33 += m * (p.X * p.X + p.Y * p.Y);

                currentInertia.M12 += -p.X * p.Y * m;
                currentInertia.M21 += -p.X * p.Y * m;

                currentInertia.M31 += -p.X * p.Z * m;
                currentInertia.M13 += -p.X * p.Z * m;

                currentInertia.M32 += -p.Y * p.Z * m;
                currentInertia.M23 += -p.Y * p.Z * m;

                Inertia += currentInertia;
                Mass += m;
            }
        }

        protected override Multishape CreateWorkingClone()
        {
            return new CompoundShape(ref _shapes);
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
            Vector3 result;

            JMath.Transform(direction, Shapes[currentShape].InverseOrientation, out result);
            result = Shapes[currentShape].Shape.SupportMapping(direction);
            JMath.Transform(result, Shapes[currentShape].Orientation, out result);
            result += Shapes[currentShape].Position;

            return result;
        }

        /// <summary>
        /// Gets the axis aligned bounding box of the orientated shape. (Inlcuding all
        /// 'sub' shapes)
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="box">The axis aligned bounding box of the shape.</param>
        public override void GetBoundingBox(Matrix4x4 orientation, out JBBox box)
        {
            box.Min = mInternalBBox.Min;
            box.Max = mInternalBBox.Max;

            Vector3 localHalfExtents = 0.5f * (box.Max - box.Min);
            Vector3 localCenter = 0.5f * (box.Max + box.Min);

            Vector3 center;
            JMath.Transform(localCenter, orientation, out center);

            Matrix4x4 abs; JMath.Absolute(orientation, out abs);
            Vector3 temp;
            JMath.Transform(localHalfExtents, abs, out temp);

            box.Max = center + temp;
            box.Min = center - temp;
        }

        int currentShape = 0;
        List<int> currentSubShapes = new List<int>();

        /// <summary>
        /// Sets the current shape. First <see cref="CompoundShape.Prepare"/> has to be called.
        /// After SetCurrentShape the shape immitates another shape.
        /// </summary>
        /// <param name="index"></param>
        public override void SetCurrentShape(int index)
        {
            currentShape = currentSubShapes[index];
            GeometricCenter = Shapes[currentShape].Shape.SupportCenter;
            GeometricCenter += Shapes[currentShape].Position;
        }

        /// <summary>
        /// Passes a axis aligned bounding box to the shape where collision
        /// could occour.
        /// </summary>
        /// <param name="box">The bounding box where collision could occur.</param>
        /// <returns>The upper index with which <see cref="SetCurrentShape"/> can be 
        /// called.</returns>
        public override int Prepare(JBBox box)
        {
            currentSubShapes.Clear();

            for (int i = 0; i < Shapes.Length; i++)
            {
                if (Shapes[i].BoundingBox.Contains(box) != JBBox.ContainmentType.Disjoint)
                    currentSubShapes.Add(i);
            }

            return currentSubShapes.Count;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="rayOrigin"></param>
        /// <param name="rayEnd"></param>
        /// <returns></returns>
        public override int Prepare(Vector3 rayOrigin, Vector3 rayEnd)
        {
            JBBox box = JBBox.SmallBox;

            box.AddPoint(rayOrigin);
            box.AddPoint(rayEnd);

            return this.Prepare(box);
        }


        public override void UpdateShape()
        {
            DoShifting();
            UpdateInternalBoundingBox();
            base.UpdateShape();
        }

        protected void UpdateInternalBoundingBox()
        {
            mInternalBBox.Min = new Vector3(float.MaxValue);
            mInternalBBox.Max = new Vector3(float.MinValue);

            for (int i = 0; i < Shapes.Length; i++)
            {
                Shapes[i].UpdateBoundingBox();

                JBBox.CreateMerged(mInternalBBox, Shapes[i].BoundingBox, out mInternalBBox);
            }
        }
    }
}
