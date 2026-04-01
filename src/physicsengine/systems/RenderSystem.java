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
        // RenderSystem doesn't participate in physics updates
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

            g2d.setColor(renderable.color);

            if (shape instanceof CircleComponent circle) {
                int x = (int)(transform.position.x - circle.radius);
                int y = (int)(transform.position.y - circle.radius);
                int d = (int)(circle.radius * 2);
                if (renderable.filled) g2d.fillOval(x, y, d, d);
                else g2d.drawOval(x, y, d, d);
            } else if (shape instanceof RectangleComponent rect) {
                Graphics2D g2dCopy = (Graphics2D) g2d.create();
                g2dCopy.translate(transform.position.x, transform.position.y);
                g2dCopy.rotate(transform.rotation);
                int x = (int)(-rect.width / 2);
                int y = (int)(-rect.height / 2);
                if (renderable.filled) g2dCopy.fillRect(x, y, (int)rect.width, (int)rect.height);
                else g2dCopy.drawRect(x, y, (int)rect.width, (int)rect.height);
                g2dCopy.dispose();
            }
        }
    }
}