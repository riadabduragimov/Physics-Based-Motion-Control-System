#include "stdafx.h"
#include "PhysicsEngine.h"

/**
 * @brief Constructs the physics engine and initializes global parameters.
 *
 * gravity  – constant downward acceleration applied to all objects
 * floorY   – y-coordinate of the ground plane used for collision detection
 */
PhysicsEngine::PhysicsEngine()
    : gravity(0.0f, -9.8f, 0.0f), floorY(0.0f) {
}


/**
 * @brief Adds a rigid body to the simulation world.
 *
 * Bodies are stored by value for simplicity. In larger engines this would
 * typically be a pointer or handle to support polymorphism and memory pools.
 */
void PhysicsEngine::addObject(const RigidBody& obj) {
    objects.push_back(obj);
}


/**
 * @brief Advances the physics simulation by one time step.
 *
 * Performs:
 *   1. Forces and integration
 *   2. Ground collision resolution
 *   3. Sphere-sphere collision detection and response
 *
 * A fixed number of substeps improves numerical stability, especially for
 * fast-moving objects or stiff interactions.
 *
 * @param dt  Simulation timestep in seconds.
 */
void PhysicsEngine::update(float dt) {
    int substeps = 5;                  // fixed physics substeps for stability
    float subDt = dt / substeps;       // timestep per substep

    for (int s = 0; s < substeps; ++s) {

        // --- Apply forces & integrate individual objects ---
        for (auto& obj : objects) {

            // Apply gravity (F = m * g)
            obj.applyForce(gravity * obj.mass);

            // Integrate linear and angular motion
            obj.update(subDt);

            // Handle collision against floor plane
            obj.handleFloorCollision(floorY);
        }

        // --- Resolve pairwise sphere-sphere collisions ---
        handleCollisions();
    }
}


/**
 * @brief Detects and resolves collisions between all pairs of rigid bodies.
 *
 * Uses:
 *   - Bounding sphere collision test
 *   - Simple impulse-based response
 *   - Positional correction to avoid sinking
 *
 * Complexity: O(n²) — sufficient for small student projects.
 */
void PhysicsEngine::handleCollisions() {
    for (size_t i = 0; i < objects.size(); ++i) {
        for (size_t j = i + 1; j < objects.size(); ++j) {
            objects[i].handleCollision(objects[j]);
        }
    }
}
