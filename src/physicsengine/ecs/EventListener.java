package physicsengine.ecs;

public interface EventListener<T extends Event> {
    void onEvent(T event);
}
