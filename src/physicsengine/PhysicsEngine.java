package physicsengine;

import physicsengine.components.*;
import physicsengine.ecs.World;
import physicsengine.systems.RenderSystem;
import physicsengine.systems.PhysicsSystem;
import physicsengine.util.Vector2;
import java.awt.Color;
import javax.swing.*;

public class PhysicsEngine {
    public static void main(String[] args) {
        // Create the world
        World world = new World();

        // Create some test entities
        int ball = world.createEntity();
        world.addComponent(ball, new TransformComponent(400, 300));
        world.addComponent(ball, new DynamicComponent(1.0f));
        world.addComponent(ball, new CollisionComponent(0.8f, 0.3f));
        world.addComponent(ball, new CircleComponent(25));
        world.addComponent(ball, new RenderableComponent(Color.BLUE, true));
        
        // Sync previous position with starting position
        TransformComponent ballTransform = world.getComponent(ball, TransformComponent.class);
        DynamicComponent ballDynamic = world.getComponent(ball, DynamicComponent.class);
        ballDynamic.previousPosition.set(ballTransform.position.x, ballTransform.position.y);

        int wall = world.createEntity();
        world.addComponent(wall, new TransformComponent(400, 550));
        world.addComponent(wall, new CollisionComponent(0.2f, 0.8f));
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
            PhysicsSystem physicsSystem = new PhysicsSystem(world);
            world.addSystem(physicsSystem);

            // Add panel to frame
            frame.add(panel);
            frame.pack();
            frame.setLocationRelativeTo(null); // center on screen
            frame.setVisible(true);

            // Start game loop
            GameLoop gameLoop = new GameLoop(world, panel);
            gameLoop.start();
        });
    }
}