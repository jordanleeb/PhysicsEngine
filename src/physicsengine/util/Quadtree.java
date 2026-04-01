package physicsengine.util;

import java.util.*;

public class Quadtree {
    private static final int MAX_CAPACITY = 8;
    private static final int MAX_DEPTH = 8;

    private final AABB bounds;
    private final int depth;
    private Quadtree[] children;

    // Each entry stores entity ID and its AABB together
    private final List<Integer> entityIds;
    private final List<AABB> entityBounds;

    public Quadtree(AABB bounds) {
        this(bounds, 0);
    }

    private Quadtree(AABB bounds, int depth) {
        this.bounds = bounds;
        this.depth = depth;
        this.entityIds = new ArrayList<>();
        this.entityBounds = new ArrayList<>();
        this.children = null;
    }

    public void insert(int entityId, AABB entityAABB) {
        // If this node has children, try to insert into appropriate child
        if (children != null) {
            int childIndex = getChildIndex(entityAABB);
            if (childIndex != -1) {
                children[childIndex].insert(entityId, entityAABB);
                return;
            }
            // Spans multiple quadrants — stays in this node
        }

        entityIds.add(entityId);
        entityBounds.add(entityAABB);

        // Subdivide if over capacity and not at max depth
        if (entityIds.size() > MAX_CAPACITY && depth < MAX_DEPTH && children == null) {
            subdivide();
            redistributeEntities();
        }
    }

    public void query(AABB queryBounds, Set<Integer> result) {
        if (!bounds.intersects(queryBounds)) return;

        for (int i = 0; i < entityIds.size(); i++) {
            if (entityBounds.get(i).intersects(queryBounds)) {
                result.add(entityIds.get(i));
            }
        }

        if (children != null) {
            for (Quadtree child : children) {
                child.query(queryBounds, result);
            }
        }
    }

    // Query for all candidate pairs that might be colliding
    public void queryPairs(List<int[]> pairs) {
        // Check all entities in this node against each other
        for (int i = 0; i < entityIds.size(); i++) {
            for (int j = i + 1; j < entityIds.size(); j++) {
                if (entityBounds.get(i).intersects(entityBounds.get(j))) {
                    pairs.add(new int[]{entityIds.get(i), entityIds.get(j)});
                }
            }
        }

        if (children != null) {
            for (Quadtree child : children) {
                // Check entities in this node against entities in children
                for (int i = 0; i < entityIds.size(); i++) {
                    Set<Integer> childResults = new HashSet<>();
                    child.query(entityBounds.get(i), childResults);
                    for (int childEntity : childResults) {
                        pairs.add(new int[]{entityIds.get(i), childEntity});
                    }
                }
                child.queryPairs(pairs);
            }
        }
    }

    private void subdivide() {
        float midX = bounds.centerX();
        float midY = bounds.centerY();

        children = new Quadtree[4];
        children[0] = new Quadtree(new AABB(bounds.minX, bounds.minY, midX, midY), depth + 1); // NW
        children[1] = new Quadtree(new AABB(midX, bounds.minY, bounds.maxX, midY), depth + 1); // NE
        children[2] = new Quadtree(new AABB(bounds.minX, midY, midX, bounds.maxY), depth + 1); // SW
        children[3] = new Quadtree(new AABB(midX, midY, bounds.maxX, bounds.maxY), depth + 1); // SE
    }

    private void redistributeEntities() {
        List<Integer> toRemove = new ArrayList<>();
        for (int i = 0; i < entityIds.size(); i++) {
            int childIndex = getChildIndex(entityBounds.get(i));
            if (childIndex != -1) {
                children[childIndex].insert(entityIds.get(i), entityBounds.get(i));
                toRemove.add(i);
            }
        }
        // Remove redistributed entities in reverse order
        for (int i = toRemove.size() - 1; i >= 0; i--) {
            int idx = toRemove.get(i);
            entityIds.remove(idx);
            entityBounds.remove(idx);
        }
    }

    private int getChildIndex(AABB entityAABB) {
        if (children == null) return -1;
        for (int i = 0; i < 4; i++) {
            if (children[i].bounds.contains(entityAABB)) {
                return i;
            }
        }
        return -1; // spans multiple quadrants
    }

    public void clear() {
        entityIds.clear();
        entityBounds.clear();
        children = null;
    }
}