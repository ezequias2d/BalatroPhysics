using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BalatroPhysics.Collision.Shapes;
using BalatroPhysics.LinearMath;
using BalatroPhysics.Dynamics;
using Microsoft.Xna.Framework;
using BalatroPhysics;
using BalatroPhysics.Dynamics.Constraints;
using BalatroPhysics.Dynamics.Joints;

namespace BalatroPhysicsDemo.Scenes
{

    public class EmptyScene : Scene
    {
       
        public EmptyScene(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();


        }
    }


}
