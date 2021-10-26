using BalatroPhysics.Collision;
using BalatroPhysics.Collision.Shapes;
using BalatroPhysics.Dynamics;
using Microsoft.Xna.Framework.Graphics;
using System.Collections.Generic;

namespace BalatroPhysicsDemo.Scenes
{
    class SoftBodyJenga : Scene
    {

        public SoftBodyJenga(JitterDemoGame demo)
            : base(demo)
        {
        }

        private void RemoveDuplicateVertices(List<TriangleVertexIndices> indices,
                List<System.Numerics.Vector3> vertices)
        {
            Dictionary<System.Numerics.Vector3, int> unique = new Dictionary<System.Numerics.Vector3, int>(vertices.Count);
            Stack<int> tbr = new Stack<int>(vertices.Count / 3);

            // get all unique vertices and their indices
            for (int i = 0; i < vertices.Count; i++)
            {
                if (!unique.ContainsKey(vertices[i]))
                    unique.Add(vertices[i], unique.Count);
                else tbr.Push(i);
            }

            // reconnect indices
            for (int i = 0; i < indices.Count; i++)
            {
                TriangleVertexIndices tvi = indices[i];

                tvi.I0 = unique[vertices[tvi.I0]];
                tvi.I1 = unique[vertices[tvi.I1]];
                tvi.I2 = unique[vertices[tvi.I2]];

                indices[i] = tvi;
            }

            // remove duplicate vertices
            while (tbr.Count > 0) vertices.RemoveAt(tbr.Pop());

            unique.Clear();
        }

        public override void Build()
        {
            AddGround();

            for (int i = 0; i < 15; i++)
            {
                bool even = (i % 2 == 0);

                for (int e = 0; e < 3; e++)
                {
                    System.Numerics.Vector3 size = (even) ? new System.Numerics.Vector3(1, 1, 3) : new System.Numerics.Vector3(3, 1, 1);
                    RigidBody body = new RigidBody(new BoxShape(size));
                    body.Position = new System.Numerics.Vector3(3.0f + (even ? e : 1.0f), i + 0.5f, -5.0f + (even ? 1.0f : e));

                    Demo.World.AddBody(body);
                }

            }


            Model model = this.Demo.Content.Load<Model>("torus");

            List<TriangleVertexIndices> indices = new List<TriangleVertexIndices>();
            List<System.Numerics.Vector3> vertices = new List<System.Numerics.Vector3>();

            ConvexHullObject.ExtractData(vertices, indices, model);
            RemoveDuplicateVertices(indices, vertices);

            SoftBody softBody = new SoftBody(indices, vertices);

            softBody.Translate(new System.Numerics.Vector3(10, 5, 0));
            softBody.Pressure = 1000.0f;
            softBody.SetSpringValues(0.2f, 0.005f);
            //softBody.SelfCollision = true; ;

            Demo.World.AddBody(softBody);

            SoftBody cloth = new SoftBody(20, 20, 0.4f);

            // ##### Uncomment for selfcollision, all 3 lines
            //cloth.SelfCollision = true;
            //cloth.TriangleExpansion = 0.05f;
            //cloth.VertexExpansion = 0.05f;

            cloth.Translate(new System.Numerics.Vector3(0, 10, 10));

            cloth.Material.KineticFriction = 0.9f;
            cloth.Material.StaticFriction = 0.95f;

            cloth.VertexBodies[0].IsStatic = true;
            cloth.VertexBodies[380].IsStatic = true;
            cloth.VertexBodies[19].IsStatic = true;
            cloth.VertexBodies[399].IsStatic = true;

            cloth.SetSpringValues(SoftBody.SpringType.EdgeSpring, 0.1f, 0.01f);
            cloth.SetSpringValues(SoftBody.SpringType.ShearSpring, 0.1f, 0.03f);
            cloth.SetSpringValues(SoftBody.SpringType.BendSpring, 0.1f, 0.03f);

            // ###### Uncomment here for a better visualization
            // Demo.Components.Add(new ClothObject(Demo, cloth));

            Demo.World.AddBody(cloth);
        }


    }
}