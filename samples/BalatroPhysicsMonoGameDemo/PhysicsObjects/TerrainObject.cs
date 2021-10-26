using BalatroPhysics.Collision.Shapes;
using BalatroPhysics.Dynamics;
using BalatroPhysicsDemo;
using BalatroPhysicsDemo.Primitives3D;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using System;

namespace JitterDemo
{
    public class TerrainObject : DrawableGameComponent
    {
        TerrainPrimitive primitive;
        BasicEffect effect;
        RigidBody terrainBody;

        Matrix worldMatrix = Matrix.Identity;

        public Matrix World
        {
            get { return worldMatrix; }
            set
            {
                worldMatrix = value;
                terrainBody.Orientation = Conversion.ToJitterMatrix(worldMatrix);
                terrainBody.Position = Conversion.ToJitterVector(worldMatrix.Translation);

            }
        }

        public TerrainObject(Game game, BasicEffect effect)
            : base(game)
        {
            this.effect = effect;
        }

        public override void Initialize()
        {
            base.Initialize();
            primitive = new TerrainPrimitive(GraphicsDevice,
                (int a, int b) =>
                { return (float)(Math.Sin(a * 0.1f) * Math.Cos(b * 0.1f)) * 3; });

            var demo = Game as JitterDemoGame;

            TerrainShape terrainShape = new TerrainShape(primitive.heights, 1.0f, 1.0f);

            terrainBody = new RigidBody(terrainShape);
            terrainBody.IsStatic = true;
            terrainBody.Tag = true;

            demo.World.AddBody(terrainBody);

            World = Matrix.CreateTranslation(-50, 0, -50);
        }

        public override void Draw(GameTime gameTime)
        {
            effect.DiffuseColor = Color.Red.ToVector3();
            primitive.AddWorldMatrix(worldMatrix);
            primitive.Draw(effect);
        }
    }
}
