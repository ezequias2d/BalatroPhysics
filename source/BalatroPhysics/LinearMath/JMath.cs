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

namespace BalatroPhysics.LinearMath
{

    /// <summary>
    /// Contains some math operations used within BalatroPhysics.
    /// </summary>
    public static class JMath
    {
        #region Static readonly variables
        /// <summary>
        /// A vector with components (0,0,0);
        /// </summary>
        public static readonly Vector3 Zero;
        /// <summary>
        /// A vector with components (1,0,0);
        /// </summary>
        public static readonly Vector3 Left;
        /// <summary>
        /// A vector with components (-1,0,0);
        /// </summary>
        public static readonly Vector3 Right;
        /// <summary>
        /// A vector with components (0,1,0);
        /// </summary>
        public static readonly Vector3 Up;
        /// <summary>
        /// A vector with components (0,-1,0);
        /// </summary>
        public static readonly Vector3 Down;
        /// <summary>
        /// A vector with components (0,0,1);
        /// </summary>
        public static readonly Vector3 Backward;
        /// <summary>
        /// A vector with components (0,0,-1);
        /// </summary>
        public static readonly Vector3 Forward;
        /// <summary>
        /// A vector with components (1,1,1);
        /// </summary>
        public static readonly Vector3 One;
        /// <summary>
        /// A vector with components 
        /// (float.MinValue,float.MinValue,float.MinValue);
        /// </summary>
        public static readonly Vector3 MinValue;
        /// <summary>
        /// A vector with components 
        /// (float.MaxValue,float.MaxValue,float.MaxValue);
        /// </summary>
        public static readonly Vector3 MaxValue;
        #endregion

        /// <summary>
        /// PI.
        /// </summary>
        public const float Pi = 3.1415926535f;

        public const float PiOver2 = 1.570796326794f;

        /// <summary>
        /// A small value often used to decide if numeric 
        /// results are zero.
        /// </summary>
        public const float Epsilon = 1.192092896e-012f;

        private const float ZeroEpsilonSq = JMath.Epsilon * JMath.Epsilon;

        static JMath()
        {
            One = new Vector3(1, 1, 1);
            Zero = new Vector3(0, 0, 0);
            Left = new Vector3(1, 0, 0);
            Right = new Vector3(-1, 0, 0);
            Up = new Vector3(0, 1, 0);
            Down = new Vector3(0, -1, 0);
            Backward = new Vector3(0, 0, 1);
            Forward = new Vector3(0, 0, -1);
            MinValue = new Vector3(float.MinValue);
            MaxValue = new Vector3(float.MaxValue);
        }

        /// <summary>
        /// Gets the square root.
        /// </summary>
        /// <param name="number">The number to get the square root from.</param>
        /// <returns></returns>
        #region public static float Sqrt(float number)
        public static float Sqrt(float number)
        {
            return (float)Math.Sqrt(number);
        }
        #endregion

        /// <summary>
        /// Gets the maximum number of two values.
        /// </summary>
        /// <param name="val1">The first value.</param>
        /// <param name="val2">The second value.</param>
        /// <returns>Returns the largest value.</returns>
        #region public static float Max(float val1, float val2)
        public static float Max(float val1, float val2)
        {
            return (val1 > val2) ? val1 : val2;
        }
        #endregion

        /// <summary>
        /// Gets the minimum number of two values.
        /// </summary>
        /// <param name="val1">The first value.</param>
        /// <param name="val2">The second value.</param>
        /// <returns>Returns the smallest value.</returns>
        #region public static float Min(float val1, float val2)
        public static float Min(float val1, float val2)
        {
            return (val1 < val2) ? val1 : val2;
        }
        #endregion

        /// <summary>
        /// Gets the maximum number of three values.
        /// </summary>
        /// <param name="val1">The first value.</param>
        /// <param name="val2">The second value.</param>
        /// <param name="val3">The third value.</param>
        /// <returns>Returns the largest value.</returns>
        #region public static float Max(float val1, float val2,float val3)
        public static float Max(float val1, float val2,float val3)
        {
            float max12 = (val1 > val2) ? val1 : val2;
            return (max12 > val3) ? max12 : val3;
        }
        #endregion

        /// <summary>
        /// Returns a number which is within [min,max]
        /// </summary>
        /// <param name="value">The value to clamp.</param>
        /// <param name="min">The minimum value.</param>
        /// <param name="max">The maximum value.</param>
        /// <returns>The clamped value.</returns>
        #region public static float Clamp(float value, float min, float max)
        public static float Clamp(float value, float min, float max)
        {
            value = (value > max) ? max : value;
            value = (value < min) ? min : value;
            return value;
        }
        #endregion
        
        /// <summary>
        /// Changes every sign of the matrix entry to '+'
        /// </summary>
        /// <param name="matrix">The matrix.</param>
        /// <param name="result">The absolute matrix.</param>
        #region public static void Absolute(JMatrix matrix,out JMatrix result)
        public static void Absolute(JMatrix matrix,out JMatrix result)
        {
            result.M11 = Math.Abs(matrix.M11);
            result.M12 = Math.Abs(matrix.M12);
            result.M13 = Math.Abs(matrix.M13);
            result.M21 = Math.Abs(matrix.M21);
            result.M22 = Math.Abs(matrix.M22);
            result.M23 = Math.Abs(matrix.M23);
            result.M31 = Math.Abs(matrix.M31);
            result.M32 = Math.Abs(matrix.M32);
            result.M33 = Math.Abs(matrix.M33);
        }
        #endregion

        /// <summary>
        /// Transforms a vector by the given matrix.
        /// </summary>
        /// <param name="position">The vector to transform.</param>
        /// <param name="matrix">The transform matrix.</param>
        /// <param name="result">The transformed vector.</param>
        public static void Transform(Vector3 position, JMatrix matrix, out Vector3 result)
        {
            float num0 = ((position.X * matrix.M11) + (position.Y * matrix.M21)) + (position.Z * matrix.M31);
            float num1 = ((position.X * matrix.M12) + (position.Y * matrix.M22)) + (position.Z * matrix.M32);
            float num2 = ((position.X * matrix.M13) + (position.Y * matrix.M23)) + (position.Z * matrix.M33);

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
        }

        /// <summary>
        /// Transforms a vector by the given matrix.
        /// </summary>
        /// <param name="position">The vector to transform.</param>
        /// <param name="matrix">The transform matrix.</param>
        /// <returns>The transformed vector.</returns>
        public static Vector3 Transform(Vector3 position, JMatrix matrix)
        {
            Vector3 result;
            JMath.Transform(position, matrix, out result);
            return result;
        }

        /// <summary>
        /// Transforms a vector by the transposed of the given Matrix.
        /// </summary>
        /// <param name="position">The vector to transform.</param>
        /// <param name="matrix">The transform matrix.</param>
        /// <param name="result">The transformed vector.</param>
        public static void TransposedTransform(Vector3 position, JMatrix matrix, out Vector3 result)
        {
            float num0 = ((position.X * matrix.M11) + (position.Y * matrix.M12)) + (position.Z * matrix.M13);
            float num1 = ((position.X * matrix.M21) + (position.Y * matrix.M22)) + (position.Z * matrix.M23);
            float num2 = ((position.X * matrix.M31) + (position.Y * matrix.M32)) + (position.Z * matrix.M33);

            result.X = num0;
            result.Y = num1;
            result.Z = num2;
        }

        /// <summary>
        /// Checks if the length of the vector is nearly zero.
        /// </summary>
        /// <returns>Returns true if the vector is nearly zero, otherwise false.</returns>
        public static bool IsNearlyZero(this Vector3 vector)
        {
            return (vector.LengthSquared() < ZeroEpsilonSq);
        }

        /// <summary>
        /// Swaps the components of both vectors.
        /// </summary>
        /// <param name="vector1">The first vector to swap with the second.</param>
        /// <param name="vector2">The second vector to swap with the first.</param>
        public static void Swap(Vector3 vector1, Vector3 vector2)
        {
            float temp;

            temp = vector1.X;
            vector1.X = vector2.X;
            vector2.X = temp;

            temp = vector1.Y;
            vector1.Y = vector2.Y;
            vector2.Y = temp;

            temp = vector1.Z;
            vector1.Z = vector2.Z;
            vector2.Z = temp;
        }

        /// <summary>
        /// Creates a quaternion from a matrix.
        /// </summary>
        /// <param name="matrix">A matrix representing an orientation.</param>
        /// <returns>JQuaternion representing an orientation.</returns>
        public static Quaternion CreateFromMatrix(JMatrix matrix)
        {
            Quaternion result;
            float num8 = (matrix.M11 + matrix.M22) + matrix.M33;
            if (num8 > 0f)
            {
                float num = (float)Math.Sqrt((double)(num8 + 1f));
                result.W = num * 0.5f;
                num = 0.5f / num;
                result.X = (matrix.M23 - matrix.M32) * num;
                result.Y = (matrix.M31 - matrix.M13) * num;
                result.Z = (matrix.M12 - matrix.M21) * num;
            }
            else if ((matrix.M11 >= matrix.M22) && (matrix.M11 >= matrix.M33))
            {
                float num7 = (float)Math.Sqrt((double)(((1f + matrix.M11) - matrix.M22) - matrix.M33));
                float num4 = 0.5f / num7;
                result.X = 0.5f * num7;
                result.Y = (matrix.M12 + matrix.M21) * num4;
                result.Z = (matrix.M13 + matrix.M31) * num4;
                result.W = (matrix.M23 - matrix.M32) * num4;
            }
            else if (matrix.M22 > matrix.M33)
            {
                float num6 = (float)Math.Sqrt((double)(((1f + matrix.M22) - matrix.M11) - matrix.M33));
                float num3 = 0.5f / num6;
                result.X = (matrix.M21 + matrix.M12) * num3;
                result.Y = 0.5f * num6;
                result.Z = (matrix.M32 + matrix.M23) * num3;
                result.W = (matrix.M31 - matrix.M13) * num3;
            }
            else
            {
                float num5 = (float)Math.Sqrt((double)(((1f + matrix.M33) - matrix.M11) - matrix.M22));
                float num2 = 0.5f / num5;
                result.X = (matrix.M31 + matrix.M13) * num2;
                result.Y = (matrix.M32 + matrix.M23) * num2;
                result.Z = 0.5f * num5;
                result.W = (matrix.M12 - matrix.M21) * num2;
            }
            return result;
        }
    }
}
