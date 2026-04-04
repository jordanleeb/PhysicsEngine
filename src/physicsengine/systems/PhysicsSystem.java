package physicsengine.systems;

import physicsengine.components.*;
import physicsengine.ecs.ECSSystem;
import physicsengine.ecs.World;
import physicsengine.util.Vector2;
import physicsengine.util.Quadtree;
import physicsengine.util.AABB;
import java.util.*;

public class PhysicsSystem extends ECSSystem {
    private static final float GRAVITY = 980.0f;
    
    private final float worldWidth;
    private final float worldHeight;
    
    // Track which entities have had their moment of inertia computed
    private final Set<Integer> initialized = new HashSet<>();
    
    // Quadtree for broad phase
    private Quadtree quadtree;
    
    // Holds the result of a collision test between two entities
    private class CollisionManifold {
        int entityA;
        int entityB;
        Vector2 normal;       // Points from B toward A
        float penetration;    // How far the shapes overlap
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
    
    public PhysicsSystem(World world, float worldWidth, float worldHeight) {
        super(world);
        this.worldWidth = worldWidth;
        this.worldHeight = worldHeight;
    }
    
    @Override
    public void update(double deltaTime) {
        float dt = (float) deltaTime;
        
        // All entities that have a transform, rigidbody, and shape ( are dynamic )
        Set<Integer> dynamicEntities = world.query(
            TransformComponent.class,
            RigidBodyComponent.class,
            ShapeComponent.class
        );
        
        // Initialize any new entities
        initializeNewEntities(dynamicEntities);
        
        // Apply forces
        for (int entity : dynamicEntities) {
            applyForces(entity);
        }
        
        // Integrate ( semi-implicit Euler )
        for (int entity : dynamicEntities) {
            integrate(entity, dt);
        }
        
        // Static entities: have shape and collision but no rigidbody
        Set<Integer> staticEntities = world.query(
            new Class<?>[]{ TransformComponent.class, CollisionComponent.class, ShapeComponent.class },
            RigidBodyComponent.class
        );
        
        // Broad phase: find candidate collision paris via quadtree
        List<int[]> candidatePairs = broadPhase(dynamicEntities, staticEntities);
        
        // Narrow phase: test actual geometry for candidate pairs
        List<CollisionManifold> manifolds = narrowPhase(candidatePairs);
        
        // Resolve collisions: apply impulses and correct positions
        resolveCollisions(manifolds);
    }
    
    private void initializeNewEntities(Set<Integer> dynamicEntities) {
        // Remove IDs of entities that no longer exist
        initialized.retainAll(dynamicEntities);
        
        for (int entity : dynamicEntities) {
            if (initialized.contains(entity)) continue;
            
            RigidBodyComponent rigid = world.getComponent(entity, RigidBodyComponent.class);
            ShapeComponent shape = world.getComponent(entity, ShapeComponent.class);
            
            // Compute moment of inertia based on shape and mass
            float mass = rigid.getMass();
            float moi = 0;
            
            if (shape instanceof CircleComponent circle) {
                // Solid disk: I = (1/2) * m * r^2
                moi = 0.5f * mass * circle.radius * circle.radius;
            } else if (shape instanceof RectangleComponent rect) {
                // Solid rectangle: I = (1/12) * m * (w^2 + h^2)
                moi = (1.0f / 12.0f) * mass * (rect.width * rect.width + rect.height * rect.height);
            }
            
            rigid.setMomentOfInertia(moi);
            initialized.add(entity);
        }
    }
    
    private void applyForces(int entity) {
        RigidBodyComponent rigid = world.getComponent(entity, RigidBodyComponent.class);
        
        // Gravity: F = m * g, applied downward (+Y is down in screen space)
        rigid.force.y += GRAVITY * rigid.getMass();
    }
    
    private void integrate(int entity, float dt) {
        RigidBodyComponent rigid = world.getComponent(entity, RigidBodyComponent.class);
        TransformComponent transform = world.getComponent(entity, TransformComponent.class);
        
        // Update velocity from acceleration: v += (F / m) * dt
        // getInverseMass() is 1/m, so this is v += F * (1/m) * dt
        rigid.velocity.x += rigid.force.x * rigid.getInverseMass() * dt;
        rigid.velocity.y += rigid.force.y * rigid.getInverseMass() * dt;
        
        // Update angular velocity from torque: ω += (τ / I) * dt
        rigid.angularVelocity += rigid.torque * rigid.getInverseMomentOfInertia() * dt;
 
        // Update position from velocity: p += v * dt
        // This uses the NEW velocity (updated above), which is what makes it semi-implicit
        transform.position.x += rigid.velocity.x * dt;
        transform.position.y += rigid.velocity.y * dt;
 
        // Update rotation from angular velocity: θ += ω * dt
        transform.rotation += rigid.angularVelocity * dt;
 
        // Clear force and torque accumulators for next frame
        rigid.force.set(0, 0);
        rigid.torque = 0;
    }
    
    private List<int[]> broadPhase(Set<Integer> dynamicEntities, Set<Integer> staticEntities) {
        // Rebuild quadtree each frame
        quadtree = new Quadtree(new AABB(0, 0, worldWidth, worldHeight));
        
        for (int entity : dynamicEntities) {
            quadtree.insert(entity, computeAABB(entity));
        }
        for (int entity : staticEntities) {
            quadtree.insert(entity, computeAABB(entity));
        }
        
        // Ask quadtree for all pairs whose AABBs overlap
        List<int[]> pairs = new ArrayList<>();
        quadtree.queryPairs(pairs);
        return pairs;
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
            // Rotated rectangle: project extents onto screen axes
            float hw = rect.width / 2;
            float hh = rect.height / 2;
            float absCos = (float) Math.abs(Math.cos(transform.rotation));
            float absSin = (float) Math.abs(Math.sin(transform.rotation));
            float extentX = hw * absCos + hh * absSin;
            float extentY = hw * absSin + hh * absCos;

            return new AABB(
                transform.position.x - extentX,
                transform.position.y - extentY,
                transform.position.x + extentX,
                transform.position.y + extentY
            );
        }
        
        return new AABB(
            transform.position.x, transform.position.y,
            transform.position.x, transform.position.y
        );
    }
    
    private List<CollisionManifold> narrowPhase(List<int[]> candidatePairs) {
        List<CollisionManifold> manifolds = new ArrayList<>();
        
        for (int[] pair : candidatePairs) {
            CollisionManifold manifold = detectCollision(pair[0], pair[1]);
            if (manifold.colliding) {
                manifolds.add(manifold);
            }
        }
        
        return manifolds;
    }
    
    private CollisionManifold detectCollision(int entityA, int entityB) {
        CollisionManifold manifold = new CollisionManifold(entityA, entityB);
        
        ShapeComponent shapeA = world.getComponent(entityA, ShapeComponent.class);
        ShapeComponent shapeB = world.getComponent(entityB, ShapeComponent.class);

        if (shapeA instanceof CircleComponent circleA && shapeB instanceof CircleComponent circleB) {
            detectCircleCircle(manifold, circleA, circleB);
        } else if (shapeA instanceof CircleComponent circleA && shapeB instanceof RectangleComponent rectB) {
            detectCircleRectangle(manifold, circleA, rectB, false);
        } else if (shapeA instanceof RectangleComponent rectA && shapeB instanceof CircleComponent circleB) {
            // Reuse circle-rectangle but tell it the order is swapped
            detectCircleRectangle(manifold, circleB, rectA, true);
        } else if (shapeA instanceof RectangleComponent rectA && shapeB instanceof RectangleComponent rectB) {
            detectRectangleRectangle(manifold, rectA, rectB);
        }

        return manifold;
    }
    
    private void detectCircleCircle(CollisionManifold manifold,
                                    CircleComponent circleA,
                                    CircleComponent circleB) {
        TransformComponent transformA = world.getComponent(manifold.entityA, TransformComponent.class);
        TransformComponent transformB = world.getComponent(manifold.entityB, TransformComponent.class);

        float dx = transformB.position.x - transformA.position.x;
        float dy = transformB.position.y - transformA.position.y;
        float distanceSquared = dx * dx + dy * dy;
        float radiiSum = circleA.radius + circleB.radius;

        if (distanceSquared >= radiiSum * radiiSum) return;

        float distance = (float) Math.sqrt(distanceSquared);
        manifold.colliding = true;

        if (distance == 0) {
            // When exactly overlapping: push apart in arbitrary direction
            manifold.normal.set(1, 0);
            manifold.penetration = circleA.radius;
            manifold.contactPoint.set(transformA.position.x, transformA.position.y);
        } else {
            manifold.normal.set(dx / distance, dy / distance);
            manifold.penetration = radiiSum - distance;
            manifold.contactPoint.set(
                transformA.position.x + manifold.normal.x * circleA.radius,
                transformA.position.y + manifold.normal.y * circleA.radius
            );
        }
    }
    
    private void detectCircleRectangle(CollisionManifold manifold,
                                       CircleComponent circle,
                                       RectangleComponent rect,
                                       boolean swapped) {
        // When swapped is true, entityA is the rectangle and entityB is the circle
        // When false, entityA is the circle and entityB is the rectangle
        int circleEntity = swapped ? manifold.entityB : manifold.entityA;
        int rectEntity = swapped ? manifold.entityA : manifold.entityB;

        TransformComponent circleTransform = world.getComponent(circleEntity, TransformComponent.class);
        TransformComponent rectTransform = world.getComponent(rectEntity, TransformComponent.class);

        // Transform circle center into rectangle's local space
        float dx = circleTransform.position.x - rectTransform.position.x;
        float dy = circleTransform.position.y - rectTransform.position.y;
        float cos = (float) Math.cos(-rectTransform.rotation);
        float sin = (float) Math.sin(-rectTransform.rotation);
        float localX = dx * cos - dy * sin;
        float localY = dx * sin + dy * cos;

        float hw = rect.width / 2;
        float hh = rect.height / 2;

        // Clamp to rectangle bounds, finding the closest point on the rect
        float closestX = Math.max(-hw, Math.min(hw, localX));
        float closestY = Math.max(-hh, Math.min(hh, localY));

        float distX = localX - closestX;
        float distY = localY - closestY;
        float distanceSquared = distX * distX + distY * distY;

        if (distanceSquared >= circle.radius * circle.radius) return;

        float distance = (float) Math.sqrt(distanceSquared);
        manifold.colliding = true;

        // Compute normal in local space, then rotate back to world space
        float cosBack = (float) Math.cos(rectTransform.rotation);
        float sinBack = (float) Math.sin(rectTransform.rotation);

        if (distance == 0) {
            float overlapX = hw - Math.abs(localX);
            float overlapY = hh - Math.abs(localY);

            float normalLocalX, normalLocalY;
            if (overlapX < overlapY) {
                manifold.penetration = overlapX + circle.radius;
                normalLocalX = localX > 0 ? 1 : -1;
                normalLocalY = 0;
            } else {
                manifold.penetration = overlapY + circle.radius;
                normalLocalX = 0;
                normalLocalY = localY > 0 ? 1 : -1;
            }

            manifold.normal.set(
                normalLocalX * cosBack - normalLocalY * sinBack,
                normalLocalX * sinBack + normalLocalY * cosBack
            );
        } else {
            manifold.penetration = circle.radius - distance;

            float normalLocalX = distX / distance;
            float normalLocalY = distY / distance;
            manifold.normal.set(
                normalLocalX * cosBack - normalLocalY * sinBack,
                normalLocalX * sinBack + normalLocalY * cosBack
            );
        }

        // Normal should point from B toward A (our convention)
        // When not swapped: A=circle, B=rect, normal points toward circle (away from rect)
        // When swapped: A=rect, B=circle, we need to flip
        if (swapped) {
            manifold.normal.negate();
        }

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

        float cosA = (float) Math.cos(transformA.rotation);
        float sinA = (float) Math.sin(transformA.rotation);
        float cosB = (float) Math.cos(transformB.rotation);
        float sinB = (float) Math.sin(transformB.rotation);

        // Four potential separating axes: two face normals from each rectangle
        Vector2[] axes = {
            new Vector2(cosA, sinA),
            new Vector2(-sinA, cosA),
            new Vector2(cosB, sinB),
            new Vector2(-sinB, cosB)
        };

        Vector2[] cornersA = getRectangleCorners(transformA, rectA);
        Vector2[] cornersB = getRectangleCorners(transformB, rectB);

        float minPenetration = Float.MAX_VALUE;
        Vector2 collisionNormal = new Vector2(0, 0);

        for (Vector2 axis : axes) {
            float[] projA = projectOntoAxis(cornersA, axis);
            float[] projB = projectOntoAxis(cornersB, axis);

            float overlap = Math.min(projA[1], projB[1]) - Math.max(projA[0], projB[0]);

            if (overlap <= 0) return; // Separating axis found = no collision

            if (overlap < minPenetration) {
                minPenetration = overlap;
                collisionNormal.set(axis.x, axis.y);
            }
        }

        // Ensure normal points from B toward A
        float dirX = transformA.position.x - transformB.position.x;
        float dirY = transformA.position.y - transformB.position.y;
        if (dirX * collisionNormal.x + dirY * collisionNormal.y < 0) {
            collisionNormal.negate();
        }

        manifold.colliding = true;
        manifold.penetration = minPenetration;
        manifold.normal.set(collisionNormal.x, collisionNormal.y);
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
            float projection = corner.x * axis.x + corner.y * axis.y;
            min = Math.min(min, projection);
            max = Math.max(max, projection);
        }
        return new float[]{min, max};
    }
    
    private void resolveCollisions(List<CollisionManifold> manifolds) {
        for (CollisionManifold manifold : manifolds) {
            resolveCollision(manifold);
        }
    }
    
    private void resolveCollision(CollisionManifold manifold) {
        // Get rigid body components, null means the entity is static
        RigidBodyComponent rigidA = world.getComponent(manifold.entityA, RigidBodyComponent.class);
        RigidBodyComponent rigidB = world.getComponent(manifold.entityB, RigidBodyComponent.class);
        TransformComponent transformA = world.getComponent(manifold.entityA, TransformComponent.class);
        TransformComponent transformB = world.getComponent(manifold.entityB, TransformComponent.class);
        
        if (rigidA == null && rigidB == null) return;
        
        // Static objects have zero inverse mass and zero inverse MOI, aka they don't move
        float invMassA = rigidA != null ? rigidA.getInverseMass() : 0;
        float invMassB = rigidB != null ? rigidB.getInverseMass() : 0;
        float invMoiA = rigidA != null ? rigidA.getInverseMomentOfInertia() : 0;
        float invMoiB = rigidB != null ? rigidB.getInverseMomentOfInertia() : 0;
        
        // Compute relative velocity at the contact point
        
        // Vectors from each body's center to the contact point
        float rAx = manifold.contactPoint.x - transformA.position.x;
        float rAy = manifold.contactPoint.y - transformA.position.y;
        float rBx = manifold.contactPoint.x - transformB.position.x;
        float rBy = manifold.contactPoint.y - transformB.position.y;
        
        // Velocity at contact = linear velocity + angular contribution
        // In 2D, the angular contribution is: ω × r = (-ω * r.y, ω * r.x)
        float velAx = 0, velAy = 0, velBx = 0, velBy = 0;
        if (rigidA != null) {
            velAx = rigidA.velocity.x - rigidA.angularVelocity * rAy;
            velAy = rigidA.velocity.y + rigidA.angularVelocity * rAx;
        }
        if (rigidB != null) {
            velBx = rigidB.velocity.x - rigidB.angularVelocity * rBy;
            velBy = rigidB.velocity.y + rigidB.angularVelocity * rBx;
        }
        
        // Relative velocity of A with respect to B at the contact point
        float relVelX = velAx - velBx;
        float relVelY = velAy - velBy;
        
        // Project relative velocity onto the collision normal
        float relVelAlongNormal = relVelX * manifold.normal.x + relVelY * manifold.normal.y;
        
        // If positive, objects are already separating — don't resolve
        if (relVelAlongNormal > 0) return;
        
        // Compute and apply the normal impulse
        
        CollisionComponent collA = world.getComponent(manifold.entityA, CollisionComponent.class);
        CollisionComponent collB = world.getComponent(manifold.entityB, CollisionComponent.class);
        
        // Use minimum restitution, the less bouncy surface wins
        float restitution = Math.min(
            collA != null ? collA.restitution : 0.3f,
            collB != null ? collB.restitution : 0.3f
        );
        
        // The impulse formula accounts for how each body's mass AND rotation
        // resist the collision. The cross products (rA × n, rB × n) capture
        // how much the contact point's velocity is affected by rotation.
        float rACrossN = rAx * manifold.normal.y - rAy * manifold.normal.x;
        float rBCrossN = rBx * manifold.normal.y - rBy * manifold.normal.x;
        
        float denominator = invMassA + invMassB
            + rACrossN * rACrossN * invMoiA
            + rBCrossN * rBCrossN * invMoiB;

        if (denominator == 0) return;
        
        // The impulse magnitude
        // j = -(1 + e) * relVel·n / denominator
        float j = -(1 + restitution) * relVelAlongNormal / denominator;

        // Apply to linear velocity: v += (j * n) / m
        // Apply to angular velocity: ω += (r × (j * n)) / I
        if (rigidA != null) {
            rigidA.velocity.x += j * manifold.normal.x * invMassA;
            rigidA.velocity.y += j * manifold.normal.y * invMassA;
            rigidA.angularVelocity += rACrossN * j * invMoiA;
        }
        if (rigidB != null) {
            rigidB.velocity.x -= j * manifold.normal.x * invMassB;
            rigidB.velocity.y -= j * manifold.normal.y * invMassB;
            rigidB.angularVelocity -= rBCrossN * j * invMoiB;
        }
        
        // Friction impulse
        
        // Tangent = relative velocity with the normal component removed
        float tangentX = relVelX - relVelAlongNormal * manifold.normal.x;
        float tangentY = relVelY - relVelAlongNormal * manifold.normal.y;
        float tangentLength = (float) Math.sqrt(tangentX * tangentX + tangentY * tangentY);

        if (tangentLength > 0.0001f) {
            // Normalize the tangent
            tangentX /= tangentLength;
            tangentY /= tangentLength;

            // Friction coefficient: geometric mean of both surfaces
            float frictionA = collA != null ? collA.friction : 0.5f;
            float frictionB = collB != null ? collB.friction : 0.5f;
            float friction = (float) Math.sqrt(frictionA * frictionB);

            // Same impulse formula but along the tangent direction
            float relVelAlongTangent = relVelX * tangentX + relVelY * tangentY;
            float rACrossT = rAx * tangentY - rAy * tangentX;
            float rBCrossT = rBx * tangentY - rBy * tangentX;
            float tangentDenom = invMassA + invMassB
                + rACrossT * rACrossT * invMoiA
                + rBCrossT * rBCrossT * invMoiB;

            float jt = -relVelAlongTangent / tangentDenom;

            // Coulomb's law: friction can't exceed μ * normal force
            if (Math.abs(jt) > Math.abs(j) * friction) {
                jt = Math.signum(jt) * Math.abs(j) * friction;
            }

            if (rigidA != null) {
                rigidA.velocity.x += jt * tangentX * invMassA;
                rigidA.velocity.y += jt * tangentY * invMassA;
                rigidA.angularVelocity += rACrossT * jt * invMoiA;
            }
            if (rigidB != null) {
                rigidB.velocity.x -= jt * tangentX * invMassB;
                rigidB.velocity.y -= jt * tangentY * invMassB;
                rigidB.angularVelocity -= rBCrossT * jt * invMoiB;
            }
        }
        
        // Position correction
        
        // Nudge objects apart so they're not overlapping.
        // The slop (0.01) prevents jitter from tiny penetrations.
        // The 0.8 means we fix 80% of the overlap each frame.
        float totalInvMass = invMassA + invMassB;
        if (totalInvMass == 0) return;

        float correctionMagnitude = Math.max(manifold.penetration - 0.01f, 0.0f)
            * 0.8f / totalInvMass;

        if (rigidA != null) {
            transformA.position.x += correctionMagnitude * manifold.normal.x * invMassA;
            transformA.position.y += correctionMagnitude * manifold.normal.y * invMassA;
        }
        if (rigidB != null) {
            transformB.position.x -= correctionMagnitude * manifold.normal.x * invMassB;
            transformB.position.y -= correctionMagnitude * manifold.normal.y * invMassB;
        }
    }
}
