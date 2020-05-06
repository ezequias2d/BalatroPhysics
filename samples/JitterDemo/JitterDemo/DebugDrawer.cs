using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using BalatroPhysics.LinearMath;
namespace BalatroPhysicsDemo
{

    /// <summary>
    /// Draw axis aligned bounding boxes, points and lines.
    /// </summary>
    public class DebugDrawer : DrawableGameComponent, BalatroPhysics.IDebugDrawer
    {
        BasicEffect basicEffect;

        public DebugDrawer(Game game)
            : base(game)
        {
        }

        public override void Initialize()
        {
            base.Initialize();
            basicEffect = new BasicEffect(this.GraphicsDevice);
            basicEffect.VertexColorEnabled = true;
        }

        public void DrawLine(System.Numerics.Vector3 p0, System.Numerics.Vector3 p1, Color color)
        {
            lineIndex += 2;

            if (lineIndex == LineList.Length)
            {
                VertexPositionColor[] temp = new VertexPositionColor[LineList.Length + 50];
                LineList.CopyTo(temp, 0);
                LineList = temp;
            }

            LineList[lineIndex - 2].Color = color;
            LineList[lineIndex - 2].Position = Conversion.ToXNAVector(p0);

            LineList[lineIndex - 1].Color = color;
            LineList[lineIndex - 1].Position = Conversion.ToXNAVector(p1);
        }

        public void DrawTriangle(System.Numerics.Vector3 p0, System.Numerics.Vector3 p1, System.Numerics.Vector3 p2, Color color)
        {
            triangleIndex += 3;

            if (triangleIndex == TriangleList.Length)
            {
                VertexPositionColor[] temp = new VertexPositionColor[TriangleList.Length + 300];
                TriangleList.CopyTo(temp, 0);
                TriangleList = temp;
            }

            TriangleList[triangleIndex - 2].Color = color;
            TriangleList[triangleIndex - 2].Position = Conversion.ToXNAVector(p0);

            TriangleList[triangleIndex - 1].Color = color;
            TriangleList[triangleIndex - 1].Position = Conversion.ToXNAVector(p1);

            TriangleList[triangleIndex - 3].Color = color;
            TriangleList[triangleIndex - 3].Position = Conversion.ToXNAVector(p2);
        }

        private void SetElement(ref System.Numerics.Vector3 v, int index, float value)
        {
            if (index == 0)
                v.X = value;
            else if (index == 1)
                v.Y = value;
            else if (index == 2)
                v.Z = value;
            else
                throw new ArgumentOutOfRangeException("index");
        }

        private float GetElement(System.Numerics.Vector3 v, int index)
        {
            if (index == 0)
                return v.X;
            if (index == 1)
                return v.Y;
            if (index == 2)
                return v.Z;

            throw new ArgumentOutOfRangeException("index");
        }

        public void DrawAabb(System.Numerics.Vector3 from, System.Numerics.Vector3 to, Color color)
        {
            System.Numerics.Vector3 halfExtents = (to - from) * 0.5f;
            System.Numerics.Vector3 center = (to + from) * 0.5f;

            System.Numerics.Vector3 edgecoord = new System.Numerics.Vector3(1f, 1f, 1f), pa, pb;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    pa = new System.Numerics.Vector3(edgecoord.X * halfExtents.X, edgecoord.Y * halfExtents.Y,
                        edgecoord.Z * halfExtents.Z);
                    pa += center;

                    int othercoord = j % 3;
                    SetElement(ref edgecoord, othercoord, GetElement(edgecoord, othercoord) * -1f);
                    pb = new System.Numerics.Vector3(edgecoord.X * halfExtents.X, edgecoord.Y * halfExtents.Y,
                        edgecoord.Z * halfExtents.Z);
                    pb += center;

                    DrawLine(pa, pb, color);
                }
                edgecoord = new System.Numerics.Vector3(-1f, -1f, -1f);
                if (i < 3)
                    SetElement(ref edgecoord, i, GetElement(edgecoord, i) * -1f);
            }
        }

        public VertexPositionColor[] TriangleList = new VertexPositionColor[99];
        public VertexPositionColor[] LineList = new VertexPositionColor[50];
        
        private int lineIndex = 0;
        private int triangleIndex = 0;

        public override void Draw(GameTime gameTime)
        {
            JitterDemo demo = Game as JitterDemo;

            basicEffect.View = demo.Camera.View;
            basicEffect.Projection = demo.Camera.Projection;

  

            foreach (EffectPass pass in basicEffect.CurrentTechnique.Passes)
            {
                pass.Apply();

                if(lineIndex > 0)
                GraphicsDevice.DrawUserPrimitives<VertexPositionColor>(
                    PrimitiveType.LineList, LineList, 0, lineIndex / 2);

                if(triangleIndex > 0)
                GraphicsDevice.DrawUserPrimitives<VertexPositionColor>(
                    PrimitiveType.TriangleList, TriangleList, 0, triangleIndex / 3);
            }

            lineIndex = 0;
            triangleIndex = 0;
 
            base.Draw(gameTime);
        }


        public void DrawLine(System.Numerics.Vector3 start, System.Numerics.Vector3 end)
        {
            DrawLine(start, end, Color.Black);
        }

        public void DrawPoint(System.Numerics.Vector3 pos)
        {
           // DrawPoint(pos, Color.Red);
        }

        public Color Color { get; set; }

        public void DrawTriangle(System.Numerics.Vector3 pos1, System.Numerics.Vector3 pos2, System.Numerics.Vector3 pos3)
        {
            DrawTriangle(pos1, pos2, pos3, Color);
        }
    }
}
