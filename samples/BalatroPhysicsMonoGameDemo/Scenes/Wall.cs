using BalatroPhysics.Collision.Shapes;
using BalatroPhysics.Dynamics;

namespace BalatroPhysicsDemo.Scenes
{
    class Wall : Scene
    {

        public Wall(JitterDemoGame demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();

            //   Demo.World.SetIterations(2);

            for (int k = 0; k < 1; k++)
            {
                for (int i = 0; i < 20; i++)
                {
                    for (int e = 0; e < 20; e++)
                    {
                        RigidBody body = new RigidBody(new BoxShape(2, 1, 1));
                        body.Position = new System.Numerics.Vector3(e * 2.01f + ((i % 2 == 0) ? 1f : 0.0f), 0.5f + i * 1.0f, k * 5);
                        Demo.World.AddBody(body);
                    }
                }
            }
        }


    }
}