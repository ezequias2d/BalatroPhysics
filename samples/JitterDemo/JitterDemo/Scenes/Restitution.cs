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
    class Restitution : Scene
    {

        public Restitution(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();

            for (int i = 0; i < 11; i++)
            {
                RigidBody box = new RigidBody(new BoxShape(1,0.01f,1));
                this.Demo.World.AddBody(box);
                System.Numerics.Vector3 boxPos = new System.Numerics.Vector3(-15 + i * 3 + 1, 5, 0);

                box.Position = boxPos;
                box.IsStatic = true;

                RigidBody sphere = new RigidBody(new SphereShape(0.5f));
                this.Demo.World.AddBody(sphere);

                sphere.Position = boxPos + JMath.Up * 30;
                sphere.EnableSpeculativeContacts = true;

                // set restitution
                sphere.Material.Restitution = box.Material.Restitution = 1.0f / 10.0f * i;
                sphere.LinearVelocity = new System.Numerics.Vector3(0, 0, 0);
    

                sphere.Damping = RigidBody.DampingType.Angular;
            }

         
        }

    }
}