package physicsengine;

import physicsengine.ecs.*;
import java.util.Set;

public class PhysicsEngine {
    public static void main(String[] args) {
        World world = new World();
        EventBus eventBus = world.getEventBus();
        
        // Create a simple test event
        class TestEvent extends Event {
            public final String message;
            public TestEvent(String message) {
                this.message = message;
            }
        }
        
        // Subscribe two listeners to the same event
        eventBus.subscribe(TestEvent.class, event -> {
            System.out.println("Listener 1 received: " + event.message);
        });
        
        eventBus.subscribe(TestEvent.class, event -> {
            System.out.println("Listener 2 received: " + event.message);
        });
        
        // Publish an event
        System.out.println("Publishing event...");
        eventBus.publish(new TestEvent("Hello from EventBus!"));
        
        // Test that unsubscribed events don't cause issues
        class UnheardEvent extends Event {}
        System.out.println("Publishing unheard event...");
        eventBus.publish(new UnheardEvent());
        System.out.println("If you're seeing this, unheard events were handled cleanly.");
    }
}
