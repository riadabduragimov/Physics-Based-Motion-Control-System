#pragma once
#include "Vector3.h"

/**
 * @class RigidBody
 *
 * @brief A simplified rigid body model representing a spherical object.
 *
 * This class supports:
 *  - Linear and angular motion
 *  - External force and torque application
 *  - Sphere-sphere collision response
 *  - Ground-plane collision handling
 *
 * Physics is intentionally simplified for real-time interactive simulations
 * (e.g., student projects, demo physics engines, introductory game engines).
 */
class RigidBody {
public:

    // --- Linear motion state ---
    Vector3 position;         ///< World-space position of the rigid body
    Vector3 velocity;         ///< Linear velocity (units/sec)
    Vector3 acceleration;     ///< Linear acceleration (reset each frame)

    // --- Angular motion state ---
    Vector3 angularVelocity;       ///< Angular velocity around each axis
    Vector3 angularAcceleration;   ///< Angular acceleration (simplified)

    // --- Physical properties ---
    float mass;               ///< Mass of the object (kg)
    float radius;             ///< Sphere radius (collision volume)
    float restitution;        ///< Bounciness (0 = inelastic, 1 = perfectly elastic)
    float friction;           ///< Collision friction applied in floor contacts
    bool grounded;            ///< True if currently in contact with the ground

    /**
     * @brief Constructs a RigidBody with customizable physical properties.
     *
     * @param pos    Initial world position
     * @param m      Mass of the body
     * @param r      Radius used for sphere collision
     * @param rest   Coefficient of restitution (bounce)
     * @param fric   Coefficient of friction applied at contact
     */
    RigidBody(Vector3 pos = Vector3(0, 0, 0),
        float m = 1.0f,
        float r = 0.5f,
        float rest = 0.8f,
        float fric = 0.2f);

    /**
     * @brief Applies a force to the rigid body, modifying linear acceleration.
     *
     * Force accumulates over the current frame and is cleared during update().
     */
    void applyForce(const Vector3& force);

    /**
     * @brief Applies a torque affecting rotational acceleration.
     *
     * This implementation uses a simplified rotational model (no inertia tensor).
     */
    void applyTorque(const Vector3& torque);

    /**
     * @brief Integrates linear and angular motion using explicit Euler.
     *
     * @param dt  Time step in seconds
     */
    void update(float dt);

    /**
     * @brief Handles collision with a horizontal floor plane.
     *
     * Applies restitution to vertical velocity and friction to horizontal motion.
     *
     * @param floorY  The y-coordinate of the floor plane
     */
    void handleFloorCollision(float floorY);

    /**
     * @brief Resolves collisions between two spherical rigid bodies.
     *
     * Handles:
     *  - Overlap detection
     *  - Impulse-based collision response
     *  - Basic friction-like torque
     *  - Positional correction to prevent objects from sinking
     */
    void handleCollision(RigidBody& other);
};
