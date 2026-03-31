package physicsengine;

import physicsengine.util.Vector2;

public class PhysicsEngine {
    public static void main(String[] args) {
        Vector2 a = new Vector2(3, 4);
        Vector2 b = new Vector2(1, 2);
        
        // Test chaining
        Vector2 chained = new Vector2(1, 1).add(b).scale(2.0f).negate();
        System.out.println("Chained result: " + chained);
        
        // Test copy safety
        Vector2 sum = a.copy().add(b);
        System.out.println("a + b: " + sum);
        System.out.println("a unchanged: " + a);
        
        // Test magnitude vs magnitudeSquared
        System.out.println("magnitude of a: " + a.magnitude());
        System.out.println("magnitudeSquared of a: " + a.magnitudeSquared());
        
        // Test dot and cross
        System.out.println("a dot b: " + a.dot(b));
        System.out.println("a cross b: " + a.cross(b));
        
        // Test normalize
        Vector2 norm = a.copy().normalize();
        System.out.println("a normalized: " + norm);
        System.out.println("normalized magnitude: " + norm.magnitude());
        
        // Test zero vector normalize safety
        Vector2 zero = new Vector2(0, 0);
        System.out.println("zero normalized: " + zero.normalize());
    }
}
