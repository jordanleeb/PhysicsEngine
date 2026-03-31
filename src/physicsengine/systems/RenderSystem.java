package physicsengine.systems;

import physicsengine.components.*;
import physicsengine.ecs.ECSSystem;
import physicsengine.ecs.World;
import java.awt.*;
import java.util.Set;

public class RenderSystem extends ECSSystem {
    
    public RenderSystem(World world) {
        super(world);
    }
    
    @Override
    public void update(double deltaTime) {
        // Rendersystem doesn't update on the physics tick
        // It only renders when Swing asks via paintComponent
    }
    
    public void render(Graphics2D g2d, double alpha) {
        Set<Integer> entities = world.query(
            TransformComponent.class,
            ShapeComponent.class,
            RenderableComponent.class
        );
        
        for (int entity : entities) {
            TransformComponent transform = world.getComponent(entity, TransformComponent.class);
            ShapeComponent shape = world.getComponent(entity, ShapeComponent.class);
            RenderableComponent renderable = world.getComponent(entity, RenderableComponent.class);
            DynamicComponent dynamic = world.getComponent(entity, DynamicComponent.class);

            // Interpolate position if entity is dynamic
            float renderX, renderY;
            if (dynamic != null) {
                renderX = (float)(dynamic.previousPosition.x + 
                          alpha * (transform.position.x - dynamic.previousPosition.x));
                renderY = (float)(dynamic.previousPosition.y + 
                          alpha * (transform.position.y - dynamic.previousPosition.y));
            } else {
                renderX = transform.position.x;
                renderY = transform.position.y;
            }

            // Set color
            g2d.setColor(renderable.color);

            // Draw based on shape type
            if (shape instanceof CircleComponent circle) {
                int x = (int)(renderX - circle.radius);
                int y = (int)(renderY - circle.radius);
                int d = (int)(circle.radius * 2);
                if (renderable.filled) g2d.fillOval(x, y, d, d);
                else g2d.drawOval(x, y, d, d);
            } else if (shape instanceof RectangleComponent rect) {
                int x = (int)(renderX - rect.width / 2);
                int y = (int)(renderY - rect.height / 2);
                if (renderable.filled) g2d.fillRect(x, y, (int)rect.width, (int)rect.height);
                else g2d.drawRect(x, y, (int)rect.width, (int)rect.height);
            }
        }
    }
}
