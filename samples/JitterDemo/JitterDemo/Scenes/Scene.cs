using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using BalatroPhysics.Dynamics;
using BalatroPhysics.LinearMath;
using BalatroPhysics.Collision.Shapes;
using BalatroPhysicsDemo.Vehicle;

namespace BalatroPhysicsDemo.Scenes
{
    public abstract class Scene
    {
        public JitterDemo Demo { get; private set; }

        public Scene(JitterDemo demo)
        {
            this.Demo = demo;
        }

        public abstract void Build();

        private QuadDrawer quadDrawer = null;
        protected RigidBody ground = null;
        protected CarObject car = null;

        public void AddGround()
        {
            ground = new RigidBody(new BoxShape(new System.Numerics.Vector3(200, 20, 200)));
            ground.Position = new System.Numerics.Vector3(0, -10, 0);
            ground.Tag = BodyTag.DontDrawMe;
            ground.IsStatic = true; Demo.World.AddBody(ground);
            //ground.Restitution = 1.0f;
            ground.Material.KineticFriction = 0.0f;

            quadDrawer = new QuadDrawer(Demo,100);
            Demo.Components.Add(quadDrawer);
        }

        public void RemoveGround()
        {
            Demo.World.RemoveBody(ground);
            Demo.Components.Remove(quadDrawer);
            quadDrawer.Dispose();
        }

        public void AddCar(System.Numerics.Vector3 position)
        {
            car = new CarObject(Demo);
            this.Demo.Components.Add(car);

            car.carBody.Position = position;
        }

        public void RemoveCar()
        {
            Demo.World.RemoveBody(car.carBody);
            Demo.Components.Remove(quadDrawer);
            Demo.Components.Remove(car);
        }


        public virtual void Draw() { }

    }
}
