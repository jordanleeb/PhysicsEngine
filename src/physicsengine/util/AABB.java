package physicsengine.util;

public class AABB {
    public float minX, minY, maxX, maxY;
    
    public AABB(float minX, float minY, float maxX, float maxY) {
        this.minX = minX;
        this.minY = minY;
        this.maxX = maxX;
        this.maxY = maxY;
    }
    
    public boolean intersects(AABB other) {
        return minX < other.maxX && maxX > other.minX &&
               minY < other.maxY && maxY > other.minY;
    }
    
    public boolean contains(AABB other) {
        return other.minX >= minX && other.maxX <= maxX &&
               other.minY >= minY && other.maxY <= maxY;
    }
    
    public float centerX() { return (minX + maxX) / 2; }
    public float centerY() { return (minY + maxY) / 2; }
    public float width()   { return maxX - minX; }
    public float height()  { return maxY - minY; }
}
