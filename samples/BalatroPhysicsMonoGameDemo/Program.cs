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
            using var game = new JitterDemoGame();
            game.Run();
        }
    }
}

