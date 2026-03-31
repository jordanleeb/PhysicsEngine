package physicsengine.components;

import physicsengine.ecs.Component;
import physicsengine.util.Vector2;

public class DynamicComponent extends Component {
    // Verlet pervious state
    public Vector2 previousPosition;
    public float previousRotation;
    
    // Encapsulated derived data
    private float mass;
    private float inverseMass;
    private float momentOfInertia;
    private float inverseInertia;
    
    // Force accumulators, cleared every physics step
    public Vector2 force;
    public float torque;
    
    public DynamicComponent(float mass) {
        this.previousPosition = new Vector2(0, 0);
        this.previousRotation = 0;
        this.force = new Vector2(0, 0);
        this.torque = 0;
        setMass(mass);
        // momentOfInertia set later by EntityFactory
    }
    
    public float getMass() { return mass; }
    public float getInverseMass() { return inverseMass; }
    public float getMomentOfInertia() { return momentOfInertia; }
    public float getInverseInertia() { return inverseInertia; }
    
    public void setMass(float mass) {
        this.mass = mass;
        this.inverseMass = mass > 0 ? 1.0f / mass : 0.0f;
    }
    
    public void setMomentOfInertia(float moi) {
        this.momentOfInertia = moi;
        this.inverseInertia = moi > 0 ? 1.0f / moi : 0.0f;
    }
}
