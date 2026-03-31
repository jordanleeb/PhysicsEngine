package physicsengine.components;

import physicsengine.ecs.Component;

public class CollisionComponent extends Component {
    public float restitution;
    public float friction;
    
    public CollisionComponent(float restitution, float friction) {
        this.restitution = restitution;
        this.friction = friction;
    }
}
