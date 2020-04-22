using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BalatroPhysics;
using Microsoft.Xna.Framework;
using BalatroPhysics.Collision.Shapes;
using BalatroPhysics.Dynamics;
using BalatroPhysics.LinearMath;

namespace BalatroPhysicsDemo.Scenes
{
    class Wall : Scene
    {

        public Wall(JitterDemo demo)
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
                        body.Position = new JVector(e * 2.01f + ((i % 2 == 0) ? 1f : 0.0f), 0.5f + i * 1.0f, k * 5);
                        Demo.World.AddBody(body);
                    }
                }
            }
        }


    }
}