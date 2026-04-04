package physicsengine;

import physicsengine.components.*;
import physicsengine.ecs.World;
import physicsengine.systems.RenderSystem;
import physicsengine.systems.PhysicsSystem;
import physicsengine.util.Vector2;
import java.awt.Color;
import javax.swing.*;

public class PhysicsEngine {
    private static final double PHYSICS_STEP = 1.0 / 60.0;
    
    public static void main(String[] args) {
        // Create the world
        World world = new World();

        // Create some test entities
        int ball = world.createEntity();
        world.addComponent(ball, new TransformComponent(400, 300));
        world.addComponent(ball, new RigidBodyComponent(1.0f));
        world.addComponent(ball, new CollisionComponent(1f, 0.3f));
        world.addComponent(ball, new CircleComponent(25));
        world.addComponent(ball, new RenderableComponent(Color.BLUE, true));

        int wall = world.createEntity();
        world.addComponent(wall, new TransformComponent(400, 550));
        world.addComponent(wall, new CollisionComponent(1f, 0.8f));
        world.addComponent(wall, new RectangleComponent(800, 20));
        world.addComponent(wall, new RenderableComponent(Color.GRAY, true));

        // Set up Swing on the EDT
        SwingUtilities.invokeLater(() -> {
            // Create window
            JFrame frame = new JFrame("Physics Sandbox");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

            // Create panel and render system
            SimulationPanel panel = new SimulationPanel();
            RenderSystem renderSystem = new RenderSystem(world);
            panel.setRenderSystem(renderSystem);
            
            // Register systems with world
            PhysicsSystem physicsSystem = new PhysicsSystem(world, 800, 600);
            world.addSystem(physicsSystem);

            // Add panel to frame
            frame.add(panel);
            frame.pack();
            frame.setLocationRelativeTo(null); // center on screen
            frame.setVisible(true);

            // Simple single threaded game loop via Swing Timer
            javax.swing.Timer gameTimer = new javax.swing.Timer(16, e -> {
                world.update(PHYSICS_STEP);
                panel.repaint();
            });
            gameTimer.start();
        });
    }
}