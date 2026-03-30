package physicsengine.ecs;

public abstract class ECSSystem {
    protected World world;
    
    public ECSSystem(World world) {
        this.world = world;
    }
    
    public abstract void update(double deltaTime);
}
