package physicsengine;

import physicsengine.ecs.World;
import physicsengine.components.*;
import physicsengine.util.Vector2;
import java.awt.Color;

public class PhysicsEngine {
    public static void main(String[] args) {
        World world = new World();
        
        // Create a dynamic ball
        int ball = world.createEntity();
        world.addComponent(ball, new TransformComponent(100, 100));
        world.addComponent(ball, new DynamicComponent(1.0f));
        world.addComponent(ball, new CollisionComponent(0.8f, 0.3f));
        world.addComponent(ball, new CircleComponent(25));
        world.addComponent(ball, new RenderableComponent(Color.BLUE, true));

        // Create a static wall
        int wall = world.createEntity();
        world.addComponent(wall, new TransformComponent(400, 500));
        world.addComponent(wall, new CollisionComponent(0.2f, 0.8f));
        world.addComponent(wall, new RectangleComponent(800, 20));
        world.addComponent(wall, new RenderableComponent(Color.GRAY, true));

        // Verify ball components
        TransformComponent ballTransform = world.getComponent(ball, TransformComponent.class);
        DynamicComponent ballDynamic = world.getComponent(ball, DynamicComponent.class);
        CircleComponent ballCircle = world.getComponent(ball, CircleComponent.class);

        System.out.println("Ball position: " + ballTransform.position);
        System.out.println("Ball mass: " + ballDynamic.getMass());
        System.out.println("Ball inverseMass: " + ballDynamic.getInverseMass());
        System.out.println("Ball radius: " + ballCircle.radius);

        // Verify wall has no DynamicComponent
        DynamicComponent wallDynamic = world.getComponent(wall, DynamicComponent.class);
        System.out.println("Wall is static: " + (wallDynamic == null));

        // Test querying
        System.out.println("\nQuerying for dynamic entities:");
        for (int entity : world.query(TransformComponent.class, DynamicComponent.class)) {
            System.out.println("  Entity " + entity + " is dynamic");
        }

        System.out.println("\nQuerying for collidable entities:");
        for (int entity : world.query(TransformComponent.class, CollisionComponent.class, ShapeComponent.class)) {
            System.out.println("  Entity " + entity + " is collidable");
        }
    }
}
