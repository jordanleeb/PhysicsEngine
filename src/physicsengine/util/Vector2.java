package physicsengine.util;

public class Vector2 {
    public float x, y;
    
    public Vector2(float x, float y) {
        this.x = x;
        this.y = y;
    }
    
    public Vector2() {
        this(0, 0);
    }
    
    // Mutating operations, supports chaining
    public Vector2 add(Vector2 other) {
        this.x += other.x;
        this.y += other.y;
        return this;
    }
    
    public Vector2 sub(Vector2 other) {
        this.x -= other.x;
        this.y -= other.y;
        return this;
    }
    
    public Vector2 scale(float scalar) {
        this.x *= scalar;
        this.y *= scalar;
        return this;
    }
    
    public Vector2 negate() {
        this.x = -this.x;
        this.y = -this.y;
        return this;
    }
    
    public Vector2 normalize() {
        float mag = magnitude();
        if (mag == 0) return this; // avoid division by zero
        return this.scale(1.0f / mag);
    }
    
    public Vector2 set(float x, float y) {
        this.x = x;
        this.y = y;
        return this;
    }
    
    // Non mutating operations
    public float dot(Vector2 other) {
        return this.x * other.x + this.y * other.y;
    }
    
    public float cross(Vector2 other) {
        return this.x * other.y - this.y * other.x;
    }
    
    public float magnitudeSquared() {
        return x*x + y*y;
    }
    
    public float magnitude() {
        return (float) Math.sqrt(magnitudeSquared());
    }
    
    public Vector2 copy() {
        return new Vector2(this.x, this.y);
    }
    
    @Override
    public String toString() {
        return "Vector2(" + x + ", " + y + ")";
    }
}
