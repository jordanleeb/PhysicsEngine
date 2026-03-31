package physicsengine;

import physicsengine.ecs.World;

public class GameLoop implements Runnable {
    private static final double PHYSICS_STEP = 1.0 / 60.0; // 60hz physics
    private static final double MAX_ACCUMULATED_TIME = 0.25; // spiral of death guard
    
    private final World world;
    private final SimulationPanel panel;
    private boolean running = false;
    
    public GameLoop(World world, SimulationPanel panel) {
        this.world = world;
        this.panel = panel;
    }
    
    public void start() {
        running = true;
        Thread thread = new Thread(this);
        thread.setDaemon(true);
        thread.start();
    }
    
    public void stop() {
        running = false;
    }
    
    @Override
    public void run() {
        double lastTime = System.nanoTime() / 1e9;
        double accumulator = 0;
        
        while (running) {
            double currentTime = System.nanoTime() / 1e9;
            double deltaTime = currentTime - lastTime;
            lastTime = currentTime;
            
            // Clamp to prevent spiral of death
            accumulator += Math.min(deltaTime, MAX_ACCUMULATED_TIME);
            
            // Fixed timestep physics updates
            while (accumulator >= PHYSICS_STEP) {
                world.update(PHYSICS_STEP);
                accumulator -= PHYSICS_STEP;
            }
            
            // Interpolation factor for rendering
            double alpha = accumulator / PHYSICS_STEP;
            panel.setAlpha(alpha);
            panel.repaint();
        }
    }
}
