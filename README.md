# CS6555 – Assignment 3: Physics-Based Motion Control System

## Overview

This project implements a physics-based motion control system for rigid bodies. It simulates gravity, collisions, and simplified rotational dynamics for multiple spherical objects. The animation shows objects falling, interacting with each other, and colliding with the floor under physically motivated rules. The simulation uses OpenGL for real-time visualization.

The system meets the requirements of Assignment 3 by supporting physics-based motion, gravity, sphere-sphere collisions, floor collisions, impulse-based collision responses with restitution, simplified rotational dynamics, and real-time animated visualization.

---

## Features

- Physics-based motion of rigid bodies
- Gravity applied globally
- Sphere-sphere collision detection using bounding spheres
- Floor collision handling with restitution and friction
- Impulse-based collision response
- Simplified rotational motion for visual plausibility
- Stable time-stepping for accurate simulation
- Real-time OpenGL visualization

---

## Code Structure

The project is organized into several modules for clarity and modularity:

- **main.cpp**: Initializes OpenGL, sets up the camera and lighting, creates the scene, and manages the rendering and timer-based update loop.  
- **PhysicsEngine.h / PhysicsEngine.cpp**: Manages all objects in the simulation, implements the global update loop, applies gravity, and handles collision detection and response.  
- **RigidBody.h / RigidBody.cpp**: Stores physical parameters for each object, applies forces and torques, integrates linear and angular motion, and implements collision logic.  
- **Vector3.h**: Provides 3D vector mathematics utilities used for physics and collision computations.  
- **stdafx.h / stdafx.cpp**: Precompiled headers for Windows Visual Studio builds to speed up compilation.

---

## Physics Implementation

### Rigid Body Representation

Each rigid body is represented as a sphere and stores position, linear velocity and acceleration, mass, radius, restitution, friction, angular velocity and acceleration, and a grounded state. This format aligns with the assignment’s required physical properties.

### Linear Motion Update

Linear motion uses semi-implicit Euler integration. Velocity is first updated by adding acceleration times the timestep, and then position is updated by adding velocity times the timestep. Acceleration is cleared after each step, and linear damping is applied (`velocity *= 0.999`) to simulate air resistance and prevent runaway energy.

### Rotational Motion (Simplified)

Angular velocity is updated using angular acceleration, and torque is computed during collisions as the cross product of the collision normal and relative velocity. This simplified approach produces visually plausible spin without a full inertia tensor model.

### Forces

Gravity is applied globally as `(0, -9.8, 0)`. The force on each object is computed as `mass * gravity`, and acceleration is updated accordingly. The system is designed to allow additional forces or user interactions.

### Collision Handling

Bounding spheres are used for collision detection. When the distance between two objects is less than the sum of their radii, an impulse-based response is applied, updating velocities according to the collision normal, relative velocity, and restitution. Positional correction separates overlapping objects to stabilize stacking and reduce jitter. Floor collisions invert the vertical velocity based on restitution, and friction reduces sliding in the horizontal plane.

### Physics Time Stepping

The main loop uses sub-stepping to maintain stability. Each frame is divided into substeps (`substeps = 5`) with `subDt = dt / substeps`. During each substep, gravity is applied, motion is integrated, floor collisions are checked, and sphere-sphere collisions are resolved. Sub-stepping improves collision accuracy and prevents tunneling.

### Rendering System (OpenGL)

The simulation is rendered using GLUT with a fixed camera via `gluLookAt`. Lighting is enabled with a single light source. The floor is drawn as a gray quad, and objects are rendered as shaded spheres using `glutSolidSphere`. Animation is performed in real time with `glutTimerFunc(16, update, 0)` (~60 FPS), and the display callback redraws the scene each frame.

---

## Approximations and Simplifications

Several simplifications ensure stability and performance while meeting educational goals. Rotational dynamics are simplified by using mass instead of a full inertia tensor, and torque is applied approximately. The collision model is impulse-based with simplified friction and no continuous collision detection. All objects are represented as spheres, simplifying collision detection and response. These approximations keep the simulation stable, visually plausible, and efficient.

---

## Requirements

- Windows or Linux system
- C++ compiler (Visual Studio recommended for Windows)
- OpenGL and GLUT libraries installed

---

## Build and Run

1. Open the project in Visual Studio or your preferred C++ IDE.
2. Ensure OpenGL and GLUT libraries are linked correctly.
3. Build the project.
4. Run the executable to see the real-time physics simulation.

---

## Conclusion

This project demonstrates a complete physics-based rigid body simulation, including gravity, sphere-sphere collisions, floor collisions with restitution and friction, linear and simplified rotational motion, stable time-stepping, and real-time OpenGL visualization. The system is modular, extendable, and easy to understand, providing a solid foundation for further exploration of physics-based animation.
