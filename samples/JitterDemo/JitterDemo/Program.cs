using System;
using BalatroPhysics.Collision.Shapes;
using BalatroPhysics.LinearMath;

namespace BalatroPhysicsDemo
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        //[STAThread()]
        static void Main(string[] args)
        {
            using (JitterDemo game = new JitterDemo())
            {
                //BalatroPhysics.DynamicTree dt = new BalatroPhysics.DynamicTree();

                //JBBox jb;
                //jb.Min = System.Numerics.Vector3.Zero;
                //jb.Max = System.Numerics.Vector3.One;

                //JBBox jb2;
                //jb2.Min = System.Numerics.Vector3.Zero;
                //jb.Max = System.Numerics.Vector3.One * 2.0f;

                //dt.CreateProxy(ref jb, 1);
                //dt.CreateProxy(ref jb, 2);

                //JBBox testBox;
                //testBox.Min = System.Numerics.Vector3.Zero;
                //testBox.Max = System.Numerics.Vector3.One *20.0f;

                //dt.Query(bla, ref testBox);
                //dt.MoveProxy


                game.Run();
            }



        }

        private static bool bla(int i)
        {

            return true;
        }
    }
}

