﻿using BalatroPhysics.Collision.Shapes;
using BalatroPhysics.Dynamics;
using Microsoft.Xna.Framework;
using System;

namespace BalatroPhysicsDemo.Scenes
{
    public class Terrain : Scene
    {

        Primitives3D.TerrainPrimitive terrain;

        public Terrain(JitterDemoGame demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            terrain = new Primitives3D.TerrainPrimitive(Demo.GraphicsDevice,
                ((a, b) =>
            {
                return (float)(Math.Cos(a * 0.2f) * Math.Sin(b * 0.2f) * 2.0f);
            }));

            TerrainShape shape = new TerrainShape(terrain.heights, 1.0f, 1.0f);

            RigidBody body = new RigidBody(shape);
            body.Position -= new System.Numerics.Vector3(50, 0, 50);
            body.IsStatic = true;
            body.Tag = BodyTag.DontDrawMe;
            //body.EnableDebugDraw = true;
            Demo.World.AddBody(body);

            AddCar(new System.Numerics.Vector3(0, 4, 0));
        }

        public override void Draw()
        {
            terrain.AddWorldMatrix(Matrix.CreateTranslation(-50, 0, -50));
            Demo.BasicEffect.DiffuseColor = Color.Red.ToVector3();
            terrain.Draw(Demo.BasicEffect);
            base.Draw();
        }

    }
}
