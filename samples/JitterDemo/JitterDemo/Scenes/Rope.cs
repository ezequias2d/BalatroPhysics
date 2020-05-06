using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BalatroPhysics;
using Microsoft.Xna.Framework;
using BalatroPhysics.Collision.Shapes;
using BalatroPhysics.Dynamics;
using BalatroPhysics.LinearMath;
using BalatroPhysics.Dynamics.Constraints;

namespace BalatroPhysicsDemo.Scenes
{
    class Rope : Scene
    {

        public Rope(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();

            RigidBody last = null;

            for (int i = 0; i < 12; i++)
            {
                RigidBody body = new RigidBody(new BoxShape(System.Numerics.Vector3.One));
                body.Position = new System.Numerics.Vector3(i * 1.5f-20, 0.5f, 0);

                System.Numerics.Vector3 jpos2 = body.Position;

                Demo.World.AddBody(body);
                body.Update();

                if (last != null)
                {
                    System.Numerics.Vector3 jpos3 = last.Position;

                    System.Numerics.Vector3 dif = jpos2 - jpos3;
                    dif *= 0.5f;
                    dif = jpos2 - dif;

                    Constraint cons = new PointOnPoint(last, body, dif);
                    Demo.World.AddConstraint(cons);
                }

                last = body;
            }
           
        }


    }
}