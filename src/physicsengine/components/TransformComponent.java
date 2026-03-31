package physicsengine.components;

import physicsengine.ecs.Component;
import physicsengine.util.Vector2;

public class TransformComponent extends Component {
    public Vector2 position;
    public float rotation;
    
    public TransformComponent(float x, float y) {
        this.position = new Vector2(x, y);
        this.rotation = 0;
    }
    
    public TransformComponent(float x, float y, float rotation) {
        this.position = new Vector2(x, y);
        this.rotation = rotation;
    }
}
