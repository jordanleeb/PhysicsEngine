package physicsengine.systems;

import physicsengine.components.*;
import physicsengine.ecs.ECSSystem;
import physicsengine.ecs.World;
import physicsengine.util.Vector2;
import physicsengine.util.Quadtree;
import physicsengine.util.AABB;
import java.util.*;

public class PhysicsSystem extends ECSSystem {
    private static final float POSITION_SLOP = 0.08f;
    private static final float POSITION_CORRECTION_PERCENT = 0.8f;
    private static final float GRAVITY = 980.0f;
    private static final int CONSTRAINT_ITERATIONS = 10;
    private static final float EPSILON = 0.001f;
    private Quadtree quadtree;

    // Internal particle representation
    private class Particle {
        Vector2 position;
        Vector2 previousPosition;

        Particle(float x, float y) {
            this.position = new Vector2(x, y);
            this.previousPosition = new Vector2(x, y);
        }
    }

    // Internal constraint representation
    private class Constraint {
        Particle a, b;
        float restLength;

        Constraint(Particle a, Particle b) {
            this.a = a;
            this.b = b;
            this.restLength = a.position.copy().sub(b.position).magnitude();
        }
    }
    
    private class CollisionManifold {
        int entityA;
        int entityB;
        Vector2 normal;
        float penetration;
        Vector2 contactPoint;
        boolean colliding;

        CollisionManifold(int entityA, int entityB) {
            this.entityA = entityA;
            this.entityB = entityB;
            this.colliding = false;
            this.normal = new Vector2(0, 0);
            this.contactPoint = new Vector2(0, 0);
            this.penetration = 0;
        }
    }

    // Per body internal state
    private final Map<Integer, List<Particle>> bodyParticles = new HashMap<>();
    private final Map<Integer, List<Constraint>> bodyConstraints = new HashMap<>();

    public PhysicsSystem(World world, float worldWidth, float worldHeight) {
        super(world);
        this.quadtree = new Quadtree(new AABB(0, 0, worldWidth, worldHeight));
    }

    @Override
    public void update(double deltaTime) {
        Set<Integer> currentEntities = world.query(
            TransformComponent.class,
            RigidBodyComponent.class,
            ShapeComponent.class
        );

        // Sync entities
        syncEntities(currentEntities);

        // Sync particles FROM components
        for (int entity : currentEntities) {
            syncParticlesFromComponents(entity, deltaTime);
        }

        // Apply forces
        for (int entity : currentEntities) {
            applyForces(entity, deltaTime);
        }

        // Integrate
        for (int entity : currentEntities) {
            integrate(entity, deltaTime);
        }

        // Solve constraints
        for (int i = 0; i < CONSTRAINT_ITERATIONS; i++) {
            for (int entity : currentEntities) {
                solveConstraints(entity);
            }
        }
        
        // Rebuild quadtree
        rebuildQuadtree();
        
        // Collision detection + response
        detectAndResolveCollisions();

        // Sync components FROM particles
        for (int entity : currentEntities) {
            syncComponentsFromParticles(entity, deltaTime);
        }
    }

    private void syncEntities(Set<Integer> currentEntities) {
        // Add new entities
        Set<Integer> toAdd = new HashSet<>(currentEntities);
        toAdd.removeAll(bodyParticles.keySet());
        for (int entity : toAdd) {
            createInternalRepresentation(entity);
        }

        // Remove deleted entities
        Set<Integer> toRemove = new HashSet<>(bodyParticles.keySet());
        toRemove.removeAll(currentEntities);
        for (int entity : toRemove) {
            bodyParticles.remove(entity);
            bodyConstraints.remove(entity);
        }
    }

    private void createInternalRepresentation(int entity) {
        TransformComponent transform = world.getComponent(entity, TransformComponent.class);
        ShapeComponent shape = world.getComponent(entity, ShapeComponent.class);

        List<Particle> particles = new ArrayList<>();
        List<Constraint> constraints = new ArrayList<>();

        if (shape instanceof CircleComponent) {
            // Circle — one center particle
            particles.add(new Particle(transform.position.x, transform.position.y));
        } else if (shape instanceof RectangleComponent rect) {
            // Rectangle — four corner particles
            float hw = rect.width / 2;
            float hh = rect.height / 2;
            float cos = (float) Math.cos(transform.rotation);
            float sin = (float) Math.sin(transform.rotation);

            // Compute rotated corners
            float[][] offsets = {{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}};
            for (float[] offset : offsets) {
                float rx = offset[0] * cos - offset[1] * sin;
                float ry = offset[0] * sin + offset[1] * cos;
                particles.add(new Particle(
                    transform.position.x + rx,
                    transform.position.y + ry
                ));
            }

            // Six constraints for rigidity
            int[][] pairs = {{0,1},{1,2},{2,3},{3,0},{0,2},{1,3}};
            for (int[] pair : pairs) {
                constraints.add(new Constraint(particles.get(pair[0]), particles.get(pair[1])));
            }
        }

        bodyParticles.put(entity, particles);
        bodyConstraints.put(entity, constraints);
    }

    private void syncParticlesFromComponents(int entity, double deltaTime) {
        TransformComponent transform = world.getComponent(entity, TransformComponent.class);
        RigidBodyComponent rigid = world.getComponent(entity, RigidBodyComponent.class);
        ShapeComponent shape = world.getComponent(entity, ShapeComponent.class);
        List<Particle> particles = bodyParticles.get(entity);

        if (shape instanceof CircleComponent) {
            Particle p = particles.get(0);
            // Set current position from component
            p.position.set(transform.position.x, transform.position.y);
            // Set previous position to imply velocity
            p.previousPosition.set(
                transform.position.x - rigid.velocity.x * (float)deltaTime,
                transform.position.y - rigid.velocity.y * (float)deltaTime
            );
        } else if (shape instanceof RectangleComponent rect) {
            float hw = rect.width / 2;
            float hh = rect.height / 2;
            float cos = (float) Math.cos(transform.rotation);
            float sin = (float) Math.sin(transform.rotation);
            float prevCos = (float) Math.cos(transform.rotation - rigid.angularVelocity * (float)deltaTime);
            float prevSin = (float) Math.sin(transform.rotation - rigid.angularVelocity * (float)deltaTime);

            float[][] offsets = {{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}};
            for (int i = 0; i < particles.size(); i++) {
                float ox = offsets[i][0];
                float oy = offsets[i][1];

                // Current position
                float rx = ox * cos - oy * sin;
                float ry = ox * sin + oy * cos;
                particles.get(i).position.set(
                    transform.position.x + rx,
                    transform.position.y + ry
                );

                // Previous position implies velocity + angularVelocity
                float prx = ox * prevCos - oy * prevSin;
                float pry = ox * prevSin + oy * prevCos;
                particles.get(i).previousPosition.set(
                    transform.position.x - rigid.velocity.x * (float)deltaTime + prx,
                    transform.position.y - rigid.velocity.y * (float)deltaTime + pry
                );
            }
        }
    }

    private void applyForces(int entity, double deltaTime) {
        RigidBodyComponent rigid = world.getComponent(entity, RigidBodyComponent.class);
        // Apply gravity
        rigid.force.add(new Vector2(0, GRAVITY * rigid.getMass()));
    }

    private void integrate(int entity, double deltaTime) {
        RigidBodyComponent rigid = world.getComponent(entity, RigidBodyComponent.class);
        List<Particle> particles = bodyParticles.get(entity);

        // Compute acceleration from forces
        Vector2 acceleration = rigid.force.copy().scale(rigid.getInverseMass());
        float dt2 = (float)(deltaTime * deltaTime);

        // Compute centroid velocity (average of particle velocities)
        Vector2 centroidVelocity = new Vector2(0, 0);
        for (Particle p : particles) {
            centroidVelocity.add(p.position.copy().sub(p.previousPosition));
        }
        centroidVelocity.scale(1.0f / particles.size());

        // Apply same displacement to all particles
        Vector2 displacement = centroidVelocity.add(acceleration.scale(dt2));
        for (Particle p : particles) {
            Vector2 next = p.position.copy().add(displacement);
            p.previousPosition.set(p.position.x, p.position.y);
            p.position.set(next.x, next.y);
        }

        // Clear force accumulator
        rigid.force.set(0, 0);
        rigid.torque = 0;
    }

    private void solveConstraints(int entity) {
        List<Constraint> constraints = bodyConstraints.get(entity);
        for (Constraint c : constraints) {
            Vector2 delta = c.b.position.copy().sub(c.a.position);
            float currentLength = delta.magnitude();
            if (currentLength == 0) continue;

            float correction = (currentLength - c.restLength) / currentLength;
            Vector2 correctionVec = delta.scale(0.5f * correction);

            c.a.position.add(correctionVec);
            c.b.position.sub(correctionVec);
        }
    }

    private void syncComponentsFromParticles(int entity, double deltaTime) {
        TransformComponent transform = world.getComponent(entity, TransformComponent.class);
        RigidBodyComponent rigid = world.getComponent(entity, RigidBodyComponent.class);
        ShapeComponent shape = world.getComponent(entity, ShapeComponent.class);
        List<Particle> particles = bodyParticles.get(entity);

        if (particles == null) {
            return;
        }

        // Compute centroid and previous centroid
        Vector2 centroid = new Vector2(0, 0);
        Vector2 prevCentroid = new Vector2(0, 0);
        for (Particle p : particles) {
            centroid.add(p.position);
            prevCentroid.add(p.previousPosition);
        }
        centroid.scale(1.0f / particles.size());
        prevCentroid.scale(1.0f / particles.size());

        // Write position
        transform.position.set(centroid.x, centroid.y);

        // Write velocity
        if (rigid != null) {
            rigid.velocity.set(
                    (centroid.x - prevCentroid.x) / (float) deltaTime,
                    (centroid.y - prevCentroid.y) / (float) deltaTime
            );
        }

        // Write rotation and angular velocity for rectangles
        if (shape instanceof RectangleComponent) {
            Vector2 edge = particles.get(1).position.copy().sub(particles.get(0).position);
            Vector2 prevEdge = particles.get(1).previousPosition.copy().sub(particles.get(0).previousPosition);

            float currentAngle = (float) Math.atan2(edge.y, edge.x);
            float prevAngle = (float) Math.atan2(prevEdge.y, prevEdge.x);

            transform.rotation = currentAngle;
            if (rigid != null) {
                rigid.angularVelocity = (currentAngle - prevAngle) / (float) deltaTime;
            }
        }

        // Apply angular velocity to circles
        if (shape instanceof CircleComponent) {
            if (rigid != null) {
                transform.rotation += rigid.angularVelocity * (float) deltaTime;
            }
        }
    }
    
    private AABB computeAABB(int entity) {
        TransformComponent transform = world.getComponent(entity, TransformComponent.class);
        ShapeComponent shape = world.getComponent(entity, ShapeComponent.class);

        if (shape instanceof CircleComponent circle) {
            return new AABB(
                transform.position.x - circle.radius,
                transform.position.y - circle.radius,
                transform.position.x + circle.radius,
                transform.position.y + circle.radius
            );
        } else if (shape instanceof RectangleComponent rect) {
            List<Particle> particles = bodyParticles.get(entity);
            if (particles != null) {
                // Dynamic — use particle positions
                float minX = Float.MAX_VALUE, minY = Float.MAX_VALUE;
                float maxX = -Float.MAX_VALUE, maxY = -Float.MAX_VALUE;
                for (Particle p : particles) {
                    minX = Math.min(minX, p.position.x);
                    minY = Math.min(minY, p.position.y);
                    maxX = Math.max(maxX, p.position.x);
                    maxY = Math.max(maxY, p.position.y);
                }
                return new AABB(minX, minY, maxX, maxY);
            } else {
                // Static — compute from transform and dimensions
                float hw = rect.width / 2;
                float hh = rect.height / 2;
                float cos = (float) Math.abs(Math.cos(transform.rotation));
                float sin = (float) Math.abs(Math.sin(transform.rotation));
                float extentX = hw * cos + hh * sin;
                float extentY = hw * sin + hh * cos;
                return new AABB(
                    transform.position.x - extentX,
                    transform.position.y - extentY,
                    transform.position.x + extentX,
                    transform.position.y + extentY
                );
            }
        }
        return new AABB(0, 0, 0, 0);
    }
    
    private void rebuildQuadtree() {
        quadtree.clear();

        // Insert dynamic entities
        Set<Integer> dynamicEntities = world.query(
            TransformComponent.class,
            RigidBodyComponent.class,
            ShapeComponent.class
        );
        for (int entity : dynamicEntities) {
            quadtree.insert(entity, computeAABB(entity));
        }

        // Insert static entities
        Set<Integer> staticEntities = world.query(
            new Class<?>[]{ TransformComponent.class, CollisionComponent.class, ShapeComponent.class },
            RigidBodyComponent.class
        );
        for (int entity : staticEntities) {
            quadtree.insert(entity, computeAABB(entity));
        }
    }
    
    private void detectCircleCircle(CollisionManifold manifold, 
                                      CircleComponent circleA, 
                                      CircleComponent circleB) {
        TransformComponent transformA = world.getComponent(manifold.entityA, TransformComponent.class);
        TransformComponent transformB = world.getComponent(manifold.entityB, TransformComponent.class);

        Vector2 delta = transformB.position.copy().sub(transformA.position);
        float distanceSquared = delta.magnitudeSquared();
        float radiiSum = circleA.radius + circleB.radius;

        // Use magnitudeSquared to avoid sqrt until we know there's a collision
        if (distanceSquared >= radiiSum * radiiSum) return;

        float distance = (float) Math.sqrt(distanceSquared);

        manifold.colliding = true;

        if (distance == 0) {
          // Exactly overlapping, push in arbitrary direction
          manifold.normal.set(1, 0);
          manifold.penetration = circleA.radius;
          manifold.contactPoint.set(transformA.position.x, transformA.position.y);
        } else {
          manifold.normal.set(delta.x / distance, delta.y / distance);
          manifold.penetration = radiiSum - distance;
          manifold.contactPoint.set(
              transformA.position.x + manifold.normal.x * circleA.radius,
              transformA.position.y + manifold.normal.y * circleA.radius
          );
        }
    }
    
    private void detectCircleRectangle(CollisionManifold manifold,
                                         CircleComponent circle,
                                      RectangleComponent rect) {
        // When called from detectCollision with swapped entities,
        // entityA is the circle, entityB is the rectangle
        TransformComponent circleTransform = world.getComponent(manifold.entityA, TransformComponent.class);
        TransformComponent rectTransform = world.getComponent(manifold.entityB, TransformComponent.class);

        // Transform circle center into rectangle's local space
        float dx = circleTransform.position.x - rectTransform.position.x;
        float dy = circleTransform.position.y - rectTransform.position.y;

        float cos = (float) Math.cos(-rectTransform.rotation);
        float sin = (float) Math.sin(-rectTransform.rotation);

        // Rotate delta into rectangle local space
        float localX = dx * cos - dy * sin;
        float localY = dx * sin + dy * cos;

        float hw = rect.width / 2;
        float hh = rect.height / 2;

        // Find closest point on rectangle to circle center in local space
        float closestX = Math.max(-hw, Math.min(hw, localX));
        float closestY = Math.max(-hh, Math.min(hh, localY));

        float distX = localX - closestX;
        float distY = localY - closestY;
        float distanceSquared = distX * distX + distY * distY;

        if (distanceSquared >= circle.radius * circle.radius) {
            return;
        }

        float distance = (float) Math.sqrt(distanceSquared);
        manifold.colliding = true;

        if (distance == 0) {
            // Circle center is inside rectangle
            // Find shortest axis to push out
            float overlapX = hw - Math.abs(localX);
            float overlapY = hh - Math.abs(localY);

            if (overlapX < overlapY) {
                manifold.penetration = overlapX + circle.radius;
                float normalLocalX = localX > 0 ? 1 : -1;
                // Transform normal back to world space
                manifold.normal.set(
                        normalLocalX * cos + 0 * (-sin),
                        normalLocalX * (-sin) + 0 * cos
                );
            } else {
                manifold.penetration = overlapY + circle.radius;
                float normalLocalY = localY > 0 ? 1 : -1;
                manifold.normal.set(
                        0 * cos + normalLocalY * (-sin),
                        0 * (-sin) + normalLocalY * cos
                );
            }
        } else {
            manifold.penetration = circle.radius - distance;

            // Transform normal from local to world space
            float normalLocalX = distX / distance;
            float normalLocalY = distY / distance;
            manifold.normal.set(
                    normalLocalX * cos + normalLocalY * (-sin),
                    normalLocalX * (-sin) + normalLocalY * cos
            );
        }

        // Contact point is on circle surface toward rectangle
        manifold.contactPoint.set(
                circleTransform.position.x - manifold.normal.x * circle.radius,
                circleTransform.position.y - manifold.normal.y * circle.radius
        );
    }
    
    private void detectRectangleRectangle(CollisionManifold manifold,
                                         RectangleComponent rectA,
                                         RectangleComponent rectB) {
        TransformComponent transformA = world.getComponent(manifold.entityA, TransformComponent.class);
        TransformComponent transformB = world.getComponent(manifold.entityB, TransformComponent.class);

        // Get axes to test (face normals of each rectangle)
        float cosA = (float) Math.cos(transformA.rotation);
        float sinA = (float) Math.sin(transformA.rotation);
        float cosB = (float) Math.cos(transformB.rotation);
        float sinB = (float) Math.sin(transformB.rotation);

        Vector2[] axes = {
            new Vector2(cosA, sinA), // A's x axis
            new Vector2(-sinA, cosA), // A's y axis
            new Vector2(cosB, sinB), // B's x axis
            new Vector2(-sinB, cosB) // B's y axis
        };

        float minPenetration = Float.MAX_VALUE;
        Vector2 collisionNormal = new Vector2(0, 0);

        // Get corners of each rectangle
        Vector2[] cornersA = getRectangleCorners(transformA, rectA);
        Vector2[] cornersB = getRectangleCorners(transformB, rectB);

        for (Vector2 axis : axes) {
            // Project both rectangles onto axis
            float[] projA = projectOntoAxis(cornersA, axis);
            float[] projB = projectOntoAxis(cornersB, axis);

            // Check overlap
            float overlap = Math.min(projA[1], projB[1]) - Math.max(projA[0], projB[0]);

            if (overlap <= 0) {
                return; // Separating axis found no collision
            }
            if (overlap < minPenetration) {
                minPenetration = overlap;
                collisionNormal.set(axis.x, axis.y);
            }
        }

        // Ensure normal points from B to A
        Vector2 direction = transformA.position.copy().sub(transformB.position);
        if (direction.dot(collisionNormal) < 0) {
            collisionNormal.negate();
        }

        manifold.colliding = true;
        manifold.penetration = minPenetration;
        manifold.normal.set(collisionNormal.x, collisionNormal.y);

        // Contact point is midpoint between centers adjusted by penetration
        manifold.contactPoint.set(
                transformA.position.x - collisionNormal.x * minPenetration / 2,
                transformA.position.y - collisionNormal.y * minPenetration / 2
        );
    }

    private Vector2[] getRectangleCorners(TransformComponent transform, RectangleComponent rect) {
        float hw = rect.width / 2;
        float hh = rect.height / 2;
        float cos = (float) Math.cos(transform.rotation);
        float sin = (float) Math.sin(transform.rotation);

        float[][] offsets = {{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}};
        Vector2[] corners = new Vector2[4];
        for (int i = 0; i < 4; i++) {
            corners[i] = new Vector2(
                    transform.position.x + offsets[i][0] * cos - offsets[i][1] * sin,
                    transform.position.y + offsets[i][0] * sin + offsets[i][1] * cos
            );
        }
        return corners;
    }

    private float[] projectOntoAxis(Vector2[] corners, Vector2 axis) {
        float min = Float.MAX_VALUE;
        float max = -Float.MAX_VALUE;
        for (Vector2 corner : corners) {
            float projection = corner.dot(axis);
            min = Math.min(min, projection);
            max = Math.max(max, projection);
        }
        return new float[]{min, max};
    }
    
    private CollisionManifold detectCollision(int entityA, int entityB) {
        CollisionManifold manifold = new CollisionManifold(entityA, entityB);

        ShapeComponent shapeA = world.getComponent(entityA, ShapeComponent.class);
        ShapeComponent shapeB = world.getComponent(entityB, ShapeComponent.class);

        if (shapeA instanceof CircleComponent circleA && shapeB instanceof CircleComponent circleB) {
            detectCircleCircle(manifold, circleA, circleB);
        } else if (shapeA instanceof CircleComponent circleA && shapeB instanceof RectangleComponent rectB) {
            detectCircleRectangle(manifold, circleA, rectB);
        } else if (shapeA instanceof RectangleComponent rectA && shapeB instanceof CircleComponent circleB) {
            // Reuse circle-rectangle but swap entities
            detectCircleRectangle(manifold, circleB, rectA);
            manifold.normal.negate(); // flip normal since entities are swapped
        } else if (shapeA instanceof RectangleComponent rectA && shapeB instanceof RectangleComponent rectB) {
            detectRectangleRectangle(manifold, rectA, rectB);
        }

        return manifold;
    }
    
    private void resolvePositions(CollisionManifold manifold) {
        boolean aIsDynamic = world.getComponent(manifold.entityA, RigidBodyComponent.class) != null;
        boolean bIsDynamic = world.getComponent(manifold.entityB, RigidBodyComponent.class) != null;

        if (!aIsDynamic && !bIsDynamic) {
            return;
        }

        RigidBodyComponent rigidA = aIsDynamic
                ? world.getComponent(manifold.entityA, RigidBodyComponent.class) : null;
        RigidBodyComponent rigidB = bIsDynamic
                ? world.getComponent(manifold.entityB, RigidBodyComponent.class) : null;

        float inverseMassA = rigidA != null ? rigidA.getInverseMass() : 0;
        float inverseMassB = rigidB != null ? rigidB.getInverseMass() : 0;

        float totalInverseMass = inverseMassA + inverseMassB;
        if (totalInverseMass == 0) {
            return;
        }

        // Get restitution
        CollisionComponent collisionA = world.getComponent(manifold.entityA, CollisionComponent.class);
        CollisionComponent collisionB = world.getComponent(manifold.entityB, CollisionComponent.class);
        float restitutionA = collisionA != null ? collisionA.restitution : 0.3f;
        float restitutionB = collisionB != null ? collisionB.restitution : 0.3f;
        float restitution = (restitutionA + restitutionB) / 2.0f;

        // Correction with slop and percent
        float correctionMagnitude = Math.max(manifold.penetration - POSITION_SLOP, 0.0f)
                * POSITION_CORRECTION_PERCENT / totalInverseMass;
        Vector2 correction = manifold.normal.copy().scale(correctionMagnitude);

        // Push A in normal direction
        if (aIsDynamic) {
            List<Particle> particlesA = bodyParticles.get(manifold.entityA);
            Vector2 correctionA = correction.copy().scale(inverseMassA);
            for (Particle p : particlesA) {
                Vector2 velocity = p.position.copy().sub(p.previousPosition);
                p.position.add(correctionA);
                p.previousPosition.set(
                        p.position.x + velocity.x * restitution,
                        p.position.y + velocity.y * restitution
                );
            }
        }

        // Push B in opposite direction
        if (bIsDynamic) {
            List<Particle> particlesB = bodyParticles.get(manifold.entityB);
            Vector2 correctionB = correction.copy().scale(inverseMassB);
            for (Particle p : particlesB) {
                Vector2 velocity = p.position.copy().sub(p.previousPosition);
                p.position.sub(correctionB);
                p.previousPosition.set(
                        p.position.x + velocity.x * restitution,
                        p.position.y + velocity.y * restitution
                );
            }
        }
    }
    
    private void resolveAngularVelocity(CollisionManifold manifold) {
        ShapeComponent shapeA = world.getComponent(manifold.entityA, ShapeComponent.class);
        ShapeComponent shapeB = world.getComponent(manifold.entityB, ShapeComponent.class);
        RigidBodyComponent rigidA = world.getComponent(manifold.entityA, RigidBodyComponent.class);
        RigidBodyComponent rigidB = world.getComponent(manifold.entityB, RigidBodyComponent.class);
        TransformComponent transformA = world.getComponent(manifold.entityA, TransformComponent.class);
        TransformComponent transformB = world.getComponent(manifold.entityB, TransformComponent.class);

        // Handle circle A
        if (shapeA instanceof CircleComponent circleA && rigidA != null) {
            float momentOfInertia = 0.5f * rigidA.getMass() * circleA.radius * circleA.radius;
            if (momentOfInertia > 0) {
                // Lever arm from circle center to contact point
                Vector2 rA = manifold.contactPoint.copy().sub(transformA.position);
                // Angular impulse = r * (correction force along normal)
                float angularImpulse = rA.cross(manifold.normal) * manifold.penetration;
                rigidA.angularVelocity -= angularImpulse / momentOfInertia;
            }
        }

        // Handle circle B
        if (shapeB instanceof CircleComponent circleB && rigidB != null) {
            float momentOfInertia = 0.5f * rigidB.getMass() * circleB.radius * circleB.radius;
            if (momentOfInertia > 0) {
                Vector2 rB = manifold.contactPoint.copy().sub(transformB.position);
                float angularImpulse = rB.cross(manifold.normal) * manifold.penetration;
                rigidB.angularVelocity += angularImpulse / momentOfInertia;
            }
        }
    }
    
    private void detectAndResolveCollisions() {
        // Get candidate pairs from quadtree
        List<int[]> pairs = new ArrayList<>();
        quadtree.queryPairs(pairs);

        // Narrow phase: build manifolds
        List<CollisionManifold> manifolds = new ArrayList<>();
        for (int[] pair : pairs) {
            CollisionManifold manifold = detectCollision(pair[0], pair[1]);
            if (manifold.colliding) manifolds.add(manifold);
        }

        // Iterative PBD positional correction
        for (int i = 0; i < CONSTRAINT_ITERATIONS; i++) {
            for (CollisionManifold manifold : manifolds) {
                resolvePositions(manifold);
            }
        }

        // Apply angular impulses for circles
        for (CollisionManifold manifold : manifolds) {
            resolveAngularVelocity(manifold);
        }
    }
}