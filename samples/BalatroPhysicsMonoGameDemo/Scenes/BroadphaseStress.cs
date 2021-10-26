using BalatroPhysics.Collision.Shapes;
using BalatroPhysics.Dynamics;

namespace BalatroPhysicsDemo.Scenes
{
    class BroadphaseStress : Scene
    {
        public BroadphaseStress(JitterDemoGame demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            BoxShape shape = new BoxShape(System.Numerics.Vector3.One);

            // CollisionSystemBrute        170 ms
            // CollisionSystemSAP          7   ms
            // CollisionSystemPersistenSAP 1   ms

            for (int i = 0; i < 15; i++)
            {
                for (int e = 0; e < 15; e++)
                {
                    for (int k = 0; k < 15; k++)
                    {
                        RigidBody b = new RigidBody(shape);
                        Demo.World.AddBody(b);
                        b.Position = new System.Numerics.Vector3(i, e, k) * 2.0f;
                        b.AffectedByGravity = false;
                    }
                }
            }
        }


    }
}