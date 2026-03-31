package physicsengine.ecs;

import java.util.*;

public class EventBus {
    private final Map<Class<?>, List<EventListener>> listeners = new HashMap<>();
    
    public <T extends Event> void subscribe(Class<T> eventType, EventListener<T> listener) {
        listeners.computeIfAbsent(eventType, k -> new ArrayList<>())
                .add(listener);
    }
    
    public <T extends Event> void publish(T event) {
        List<EventListener> eventListeners = listeners.get(event.getClass());
        if (eventListeners == null) return;
        for (EventListener listener : eventListeners) {
            listener.onEvent(event);
        }
    }
}
