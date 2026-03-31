package physicsengine.components;

public class RectangleComponent extends ShapeComponent {
    public float width, height;
    
    public RectangleComponent(float width, float height) {
        this.width = width;
        this.height = height;
    }
}
