using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BalatroPhysics.Collision.Shapes;
using BalatroPhysics.Dynamics;
using BalatroPhysics.LinearMath;

namespace BalatroPhysicsDemo.Scenes
{
    public class Domino : Scene
    {

        public Domino(JitterDemo demo) : base(demo)
        {


        }

        public override void Build()
        {
            //this.Demo.World.Solver = BalatroPhysics.World.SolverType.Sequential;


            AddGround();


            BoxShape bShape = new BoxShape(0.5f, 4.0f, 2.0f);

            for (int i = 0; i < 10; i++)
            {
                RigidBody body = new RigidBody(bShape);
                body.Position = new System.Numerics.Vector3(i * 2.0f, 2, 0);
                this.Demo.World.AddBody(body);
            }

            ground.Material.Restitution = 0.0f;
            ground.Material.StaticFriction = 0.4f;
        }

    }
}
