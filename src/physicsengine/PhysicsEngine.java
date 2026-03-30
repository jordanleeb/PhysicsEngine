package physicsengine;

import physicsengine.ecs.*;
import java.util.Set;

public class PhysicsEngine {
    public static void main(String[] args) {
        World world = new World();
        
        // Create two entities
        int entityA = world.createEntity();
        int entityB = world.createEntity();
        
        System.out.println("Entity A ID: " + entityA);
        System.out.println("Entity B ID: " + entityB);
        
        // Add a test component
        Component testComponent = new Component() {};
        world.addComponent(entityA, testComponent);
        world.addComponent(entityB, testComponent);
        
        // Query for all entities with that component
        Set<Integer> results = world.query(testComponent.getClass());
        System.out.println("Entities with test component: " + results);
        
        // Destroy entity A
        world.destroyEntity(entityA);
        results = world.query(testComponent.getClass());
        System.out.println("After destroying A: " + results);
    }
}
