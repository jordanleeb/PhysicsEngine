package physicsengine.ecs;

import java.util.*;

public class World {
    private int nextEntityId = 0;
    private final Map<Class<?>, Map<Integer, Component>> componentStores = new HashMap<>();
    private final List<ECSSystem> systems = new ArrayList<>();
    
    // Create a new entity, return its unique ID
    public int createEntity() {
        return nextEntityId++;
    }
    
    // Add a component to an entity
    public <T extends Component> void addComponent(int entityId, T component) {
        componentStores
                .computeIfAbsent(component.getClass(), k -> new HashMap<>())
                .put(entityId, component);
    }
    
    // Get a specific component type for an entity
    public <T extends Component> T getComponent(int entityId, Class<T> type) {
        Map<Integer, Component> store = componentStores.get(type);
        if (store == null) return null;
        return type.cast(store.get(entityId));
    }
    
    // Query for all entities that have ALL of the given component types
    public Set<Integer> query(Class<?>... componentTypes) {
        Set<Integer> result = null;
        for (Class<?> type : componentTypes) {
            Map<Integer, Component> store = componentStores.get(type);
            if (store == null) return new HashSet<>();
            if (result == null) result = new HashSet<>(store.keySet());
            else result.retainAll(store.keySet());
        }
        return result == null ? new HashSet<>() : result;
    }
    
    // Remove a component from an entity
    public void removeComponent(int entityId, Class<?> type) {
        Map<Integer, Component> store = componentStores.get(type);
        if (store != null) store.remove(entityId);
    }
    
    // Destroy an entity and all its components
    public void destroyEntity(int entityId) {
        for (Map<Integer, Component> store : componentStores.values()) {
            store.remove(entityId);
        }
    }
    
    // Register a system with the world
    public void addSystem(ECSSystem system) {
        systems.add(system);
    }
    
    // Update all systems
    public void update(double deltaTime) {
        for (ECSSystem system : systems) {
            system.update(deltaTime);
        }
    }
}
