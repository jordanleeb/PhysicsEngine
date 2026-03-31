package physicsengine.components;

import physicsengine.ecs.Component;
import java.awt.Color;

public class RenderableComponent extends Component {
    public Color color;
    public boolean filled;
    
    public RenderableComponent(Color color, boolean filled) {
        this.color = color;
        this.filled = filled;
    }
}
