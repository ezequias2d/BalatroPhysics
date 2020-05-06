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
    /// Represents a terrain.
    /// </summary>
    public class TerrainShape : Multishape
    {
        private float[,] heights;
        private float scaleX, scaleZ;
        private int heightsLength0, heightsLength1;

        private int minX, maxX;
        private int minZ, maxZ;
        private int numX, numZ;

        private JBBox boundings;

        private float sphericalExpansion = 0.05f;

        private Vector3[] points;
        private Vector3 normal;

        /// <summary>
        /// Expands the triangles by the specified amount.
        /// This stabilizes collision detection for flat shapes.
        /// </summary>
        public float SphericalExpansion
        {
            get { return sphericalExpansion; }
            set { sphericalExpansion = value; }
        }

        /// <summary>
        /// Initializes a new instance of the TerrainShape class.
        /// </summary>
        /// <param name="heights">An array containing the heights of the terrain surface.</param>
        /// <param name="scaleX">The x-scale factor. (The x-space between neighbour heights)</param>
        /// <param name="scaleZ">The y-scale factor. (The y-space between neighbour heights)</param>
        public TerrainShape(float[,] heights, float scaleX, float scaleZ)
        {
            points = new Vector3[3];
            normal = JMath.Up;

            heightsLength0 = heights.GetLength(0);
            heightsLength1 = heights.GetLength(1);

            #region Bounding Box
            boundings = JBBox.SmallBox;

            for (int i = 0; i < heightsLength0; i++)
            {
                for (int e = 0; e < heightsLength1; e++)
                {
                    if (heights[i, e] > boundings.Max.Y)
                        boundings.Max.Y = heights[i, e];
                    else if (heights[i, e] < boundings.Min.Y)
                        boundings.Min.Y = heights[i, e];
                }
            }

            boundings.Min.X = 0.0f;
            boundings.Min.Z = 0.0f;

            boundings.Max.X = checked(heightsLength0 * scaleX);
            boundings.Max.Z = checked(heightsLength1 * scaleZ);

            #endregion

            this.heights = heights;
            this.scaleX = scaleX;
            this.scaleZ = scaleZ;

            UpdateShape();
        }

        private TerrainShape() 
        {
            points = new Vector3[3];
            normal = JMath.Up;
        }

 
        protected override Multishape CreateWorkingClone()
        {
            TerrainShape clone = new TerrainShape();
            clone.heights = heights;
            clone.scaleX = scaleX;
            clone.scaleZ = scaleZ;
            clone.boundings = boundings;
            clone.heightsLength0 = heightsLength0;
            clone.heightsLength1 = heightsLength1;
            clone.sphericalExpansion = sphericalExpansion;
            return clone;
        }

        /// <summary>
        /// Sets the current shape. First <see cref="Prepare"/> has to be called.
        /// After SetCurrentShape the shape immitates another shape.
        /// </summary>
        /// <param name="index"></param>
        public override void SetCurrentShape(int index)
        {
            bool leftTriangle = false;

            if (index >= numX * numZ)
            {
                leftTriangle = true;
                index -= numX * numZ;
            }

            int quadIndexX = index % numX;
            int quadIndexZ = index / numX;

            // each quad has two triangles, called 'leftTriangle' and !'leftTriangle'
            if (leftTriangle)
            {
                points[0] = new Vector3((minX + quadIndexX + 0) * scaleX, heights[minX + quadIndexX + 0, minZ + quadIndexZ + 0], (minZ + quadIndexZ + 0) * scaleZ);
                points[1] = new Vector3((minX + quadIndexX + 1) * scaleX, heights[minX + quadIndexX + 1, minZ + quadIndexZ + 0], (minZ + quadIndexZ + 0) * scaleZ);
                points[2] = new Vector3((minX + quadIndexX + 0) * scaleX, heights[minX + quadIndexX + 0, minZ + quadIndexZ + 1], (minZ + quadIndexZ + 1) * scaleZ);
            }
            else
            {
                points[0] = new Vector3((minX + quadIndexX + 1) * scaleX, heights[minX + quadIndexX + 1, minZ + quadIndexZ + 0], (minZ + quadIndexZ + 0) * scaleZ);
                points[1] = new Vector3((minX + quadIndexX + 1) * scaleX, heights[minX + quadIndexX + 1, minZ + quadIndexZ + 1], (minZ + quadIndexZ + 1) * scaleZ);
                points[2] = new Vector3((minX + quadIndexX + 0) * scaleX, heights[minX + quadIndexX + 0, minZ + quadIndexZ + 1], (minZ + quadIndexZ + 1) * scaleZ);
            }

            GeometricCenter = (points[0] + points[1] + points[2]) * (1.0f / 3.0f);

            normal = Vector3.Cross(points[1] - points[0], points[2] - points[0]);
        }

        public void CollisionNormal(out Vector3 normal)
        {
            normal = this.normal;
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
            // simple idea: the terrain is a grid. x and z is the position in the grid.
            // y the height. we know compute the min and max grid-points. All quads
            // between these points have to be checked.

            // including overflow exception prevention

            if (box.Min.X < boundings.Min.X) minX = 0;
            else
            {
                minX = (int)Math.Floor((float)((box.Min.X - sphericalExpansion) / scaleX));
                minX = Math.Max(minX, 0);
            }

            if (box.Max.X > boundings.Max.X) maxX = heightsLength0 - 1;
            else
            {
                maxX = (int)Math.Ceiling((float)((box.Max.X + sphericalExpansion) / scaleX));
                maxX = Math.Min(maxX, heightsLength0 - 1);
            }

            if (box.Min.Z < boundings.Min.Z) minZ = 0;
            else
            {
                minZ = (int)Math.Floor((float)((box.Min.Z - sphericalExpansion) / scaleZ));
                minZ = Math.Max(minZ, 0);
            }

            if (box.Max.Z > boundings.Max.Z) maxZ = heightsLength1 - 1;
            else
            {
                maxZ = (int)Math.Ceiling((float)((box.Max.Z + sphericalExpansion) / scaleZ));
                maxZ = Math.Min(maxZ, heightsLength1 - 1);
            }

            numX = maxX - minX;
            numZ = maxZ - minZ;

            // since every quad contains two triangles we multiply by 2.
            return numX * numZ * 2;
        }

        /// <summary>
        /// 
        /// </summary>
        public override void CalculateMassInertia()
        {
            Inertia = Matrix4x4.Identity;
            Mass = 1.0f;
        }

        /// <summary>
        /// Gets the axis aligned bounding box of the orientated shape. This includes
        /// the whole shape.
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="box">The axis aligned bounding box of the shape.</param>
        public override void GetBoundingBox(Matrix4x4 orientation, out JBBox box)
        {
            box = boundings;

            #region Expand Spherical
            Vector3 expansion = new Vector3(sphericalExpansion, sphericalExpansion, sphericalExpansion);
            box.Min -= expansion;
            box.Max += expansion;
            #endregion

            box.Transform(orientation);
        }

        public override void MakeHull(List<Vector3> triangleList, int generationThreshold)
        {
            for (int index = 0; index < (heightsLength0 - 1) * (heightsLength1 - 1); index++)
            {
                int quadIndexX = index % (heightsLength0 - 1);
                int quadIndexZ = index / (heightsLength0 - 1);

                triangleList.Add(new Vector3( quadIndexX      * scaleX, heights[quadIndexX    , quadIndexZ    ],  quadIndexZ      * scaleZ));
                triangleList.Add(new Vector3((quadIndexX + 1) * scaleX, heights[quadIndexX + 1, quadIndexZ    ],  quadIndexZ      * scaleZ));
                triangleList.Add(new Vector3( quadIndexX      * scaleX, heights[quadIndexX    , quadIndexZ + 1], (quadIndexZ + 1) * scaleZ));

                triangleList.Add(new Vector3((quadIndexX + 1) * scaleX, heights[quadIndexX + 1, quadIndexZ    ],  quadIndexZ      * scaleZ));
                triangleList.Add(new Vector3((quadIndexX + 1) * scaleX, heights[quadIndexX + 1, quadIndexZ + 1], (quadIndexZ + 1) * scaleZ));
                triangleList.Add(new Vector3( quadIndexX      * scaleX, heights[quadIndexX    , quadIndexZ + 1], (quadIndexZ + 1) * scaleZ));
            }
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
            Vector3 expandVector;
            expandVector = Vector3.Normalize(direction);
            expandVector *= sphericalExpansion;

            int minIndex = 0;
            float min = Vector3.Dot(points[0], direction);
            float dot = Vector3.Dot(points[1], direction);
            if (dot > min)
            {
                min = dot;
                minIndex = 1;
            }
            dot = Vector3.Dot(points[2], direction);
            if (dot > min)
            {
                minIndex = 2;
            }

            return points[minIndex] + expandVector;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="rayOrigin"></param>
        /// <param name="rayDelta"></param>
        /// <returns></returns>
        public override int Prepare(Vector3 rayOrigin, Vector3 rayDelta)
        {
            JBBox box = JBBox.SmallBox;

            #region RayEnd + Expand Spherical
            Vector3 rayEnd;
            rayEnd = Vector3.Normalize(rayDelta);
            rayEnd = rayOrigin + rayDelta + rayEnd * sphericalExpansion;
            #endregion

            box.AddPoint(rayOrigin);
            box.AddPoint(rayEnd);

            return Prepare(box);
        }
    }
}
