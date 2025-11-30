#include "stdafx.h"
#include "Vector3.h"
#include "RigidBody.h"
#include <algorithm>

/**
 * Constructor for a simple rigid body object.
 *
 * @param pos        Initial position of the body in world space
 * @param m          Mass of the rigid body
 * @param r          Radius (used for spherical collision volume)
 * @param rest       Coefficient of restitution (bounciness)
 * @param fric       Coefficient of friction acting on collisions
 */
RigidBody::RigidBody(Vector3 pos, float m, float r, float rest, float fric)
    : position(pos),
    mass(m),
    radius(r),
    velocity(0, 0, 0),
    acceleration(0, 0, 0),
    angularVelocity(0, 0, 0),
    angularAcceleration(0, 0, 0),
    restitution(rest),
    friction(fric),
    grounded(false)
{
}

/**
 * Applies a force to the rigid body.
 * Updates linear acceleration based on Newton's second law: F = m * a.
 */
void RigidBody::applyForce(const Vector3& force) {
    acceleration += force / mass;
}

/**
 * Applies a torque to the rigid body.
 * Rotational dynamics are simplified here by treating torque like force.
 */
void RigidBody::applyTorque(const Vector3& torque) {
    angularAcceleration += torque / mass; // simplified rotational inertia model
}

/**
 * Updates linear and rotational motion using explicit Euler integration.
 * Applies simple damping to stabilize the simulation.
 */
void RigidBody::update(float dt) {
    // --- Linear motion integration ---
    velocity += acceleration * dt;
    position += velocity * dt;
    acceleration = Vector3(0, 0, 0); // reset for next frame

    // --- Angular motion integration ---
    angularVelocity += angularAcceleration * dt;
    angularAcceleration = Vector3(0, 0, 0);

    // --- Air resistance (simple damping model) ---
    velocity *= 0.999f;
    angularVelocity *= 0.99f;
}

/**
 * Handles collision of the sphere with the ground plane.
 * Applies restitution to vertical velocity and friction to horizontal motion.
 *
 * @param floorY    The y-coordinate of the collision plane
 */
void RigidBody::handleFloorCollision(float floorY) {
    // Check whether the sphere penetrates the floor
    if (position.y - radius <= floorY) {

        // Correct positional penetration
        position.y = floorY + radius;

        // Bounce with restitution
        velocity.y = -velocity.y * restitution;

        // Apply friction to horizontal velocity
        velocity.x *= (1.0f - friction);
        velocity.z *= (1.0f - friction);

        grounded = true;
    }
    else {
        grounded = false;
    }
}

/**
 * Resolves sphere-sphere collision using impulse-based dynamics.
 *
 * - Detects overlap
 * - Computes collision normal and relative velocity
 * - Applies impulse proportional to relative speed and restitution
 * - Applies naive torque response
 * - Pushes objects apart to prevent sinking
 */
void RigidBody::handleCollision(RigidBody& other) {

    Vector3 delta = position - other.position;
    float dist = delta.length();
    float minDist = radius + other.radius;

    // Collision test: must overlap but also not be at zero distance (avoid division by zero)
    if (dist < minDist && dist > 0) {

        // Collision normal
        Vector3 normal = delta.normalized();

        // Relative velocity
        Vector3 relVel = velocity - other.velocity;
        float velAlongNormal = relVel.dot(normal);

        // If bodies are separating, do nothing
        if (velAlongNormal > 0) return;

        // Effective restitution for the collision
        float e = std::min(restitution, other.restitution);

        // Impulse scalar
        float j = -(1 + e) * velAlongNormal;
        j /= (1 / mass + 1 / other.mass);

        // Apply impulse
        Vector3 impulse = normal * j;
        velocity += impulse / mass;
        other.velocity -= impulse / other.mass;

        // Apply simplified torque response
        Vector3 torque = normal.cross(relVel) * 0.1f;
        applyTorque(torque);
        other.applyTorque(-torque);

        // Positional correction to prevent spheres from intersecting
        float penetration = minDist - dist;
        Vector3 correction = normal * (penetration / 2.0f);
        position += correction;
        other.position -= correction;
    }
}
