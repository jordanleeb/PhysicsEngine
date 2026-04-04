package physicsengine.components;

import physicsengine.ecs.Component;
import physicsengine.util.Vector2;

public class RigidBodyComponent extends Component {
    // High level interface of explicit velocity
    public Vector2 velocity;
    public float angularVelocity;
    
    // Force accumulators
    public Vector2 force;
    public float torque;
    
    // Encapsulated mass and moment of inertia
    private float mass;
    private float inverseMass;
    private float momentOfInertia;
    private float inverseMomentOfInertia;
    
    public RigidBodyComponent(float mass) {
        this.velocity = new Vector2(0, 0);
        this.angularVelocity = 0;
        this.force = new Vector2(0, 0);
        this.torque = 0;
        setMass(mass);
        setMomentOfInertia(0);
    }
    
    public float getMass() { return mass; }
    public float getInverseMass() { return inverseMass; }
    
    public void setMass(float mass) {
        this.mass = mass;
        this.inverseMass = mass > 0 ? 1.0f / mass : 0.0f;
    }
    
    public float getMomentOfInertia() { return momentOfInertia; }
    public float getInverseMomentOfInertia() { return inverseMomentOfInertia; }
    
    public void setMomentOfInertia(float momentOfInertia) {
        this.momentOfInertia = momentOfInertia;
        this.inverseMomentOfInertia = momentOfInertia > 0 ? 1.0f / momentOfInertia : 0.0f;
    }
}
