namespace BalatroPhysicsDemo.Scenes
{

    public class EmptyScene : Scene
    {

        public EmptyScene(JitterDemoGame demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();


        }
    }


}
