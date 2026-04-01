package physicsengine.systems;

import physicsengine.components.*;
import physicsengine.ecs.ECSSystem;
import physicsengine.ecs.World;
import physicsengine.util.Vector2;
import java.util.Set;

public class PhysicsSystem extends ECSSystem {
    private static final float GRAVITY = 980.0f; // pixels per second squared
    
    public PhysicsSystem(World world) {
        super(world);
    }
    
    @Override
    public void update(double deltaTime) {
        Set<Integer> entities = world.query(
            TransformComponent.class,
            DynamicComponent.class
        );
        
        for (int entity : entities) {
            TransformComponent transform = world.getComponent(entity, TransformComponent.class);
            DynamicComponent dynamic = world.getComponent(entity, DynamicComponent.class);
            
            // Apply gravity to force accumulator
            dynamic.force.add(new Vector2(0, GRAVITY * dynamic.getMass()));
            
            // Compute acceleration from accumulated forces
            Vector2 acceleration = dynamic.force.copy().scale(dynamic.getInverseMass());
            
            // Verlet integration
            Vector2 velocity = transform.position.copy().sub(dynamic.previousPosition);
            Vector2 nextPosition = transform.position.copy()
                .add(velocity)
                .add(acceleration.copy().scale((float)(deltaTime * deltaTime)));
            
            // Shift positions
            dynamic.previousPosition.set(transform.position.x, transform.position.y);
            transform.position.set(nextPosition.x, nextPosition.y);
            
            // Clear accumulators
            dynamic.force.set(0, 0);
            dynamic.torque = 0;
        }
    }
}
