#pragma once
#include <vector>
#include "RigidBody.h"

/**
 * @class PhysicsEngine
 *
 * @brief A simple physics simulation system handling rigid-body dynamics.
 *
 * Responsibilities:
 *   - Manage a collection of RigidBody objects
 *   - Integrate linear and angular motion over time
 *   - Apply global forces (e.g., gravity)
 *   - Resolve sphere-sphere collisions between objects
 *   - Resolve collisions with a horizontal floor plane
 *
 * This engine uses a basic substep integrator for improved stability and
 * performs O(n²) pairwise collision checks, which is suitable for small-scale
 * simulations or instructional projects.
 */
class PhysicsEngine {
public:
    std::vector<RigidBody> objects; ///< All rigid bodies in the simulation world
    Vector3 gravity;                ///< Global gravity acceleration applied to all objects
    float floorY;                   ///< Y-position of the floor plane used for collisions

    /**
     * @brief Constructs a PhysicsEngine with default environment settings.
     *
     * Initializes gravity to (0, -9.8, 0) and the floor plane at y = 0.
     */
    PhysicsEngine();

    /**
     * @brief Adds a new rigid body to the simulation.
     *
     * @param obj Reference to a RigidBody to be copied into the engine's object list.
     */
    void addObject(const RigidBody& obj);

    /**
     * @brief Advances the physics simulation by a single time step.
     *
     * This performs force application, motion integration, floor collision handling,
     * and sphere-sphere collision resolution.
     *
     * @param dt Time step in seconds.
     */
    void update(float dt);

private:
    /**
     * @brief Detects and resolves all pairwise collisions between objects.
     *
     * Uses bounding-sphere collision detection and impulse-based response.
     */
    void handleCollisions();
};
