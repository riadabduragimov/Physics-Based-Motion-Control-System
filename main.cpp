#include "stdafx.h"
#include "PhysicsEngine.h"
#include <GL/glut.h>

PhysicsEngine engine;     // Global physics engine instance
float dt = 0.01f;         // Base timestep for each simulation update

// -----------------------------------------------------------------------------
// Initializes the simulation scene by creating several rigid bodies with
// different sizes, masses, and restitution values. These will fall due to
// gravity and collide with the floor and each other.
// -----------------------------------------------------------------------------
void initScene() {
    engine.addObject(RigidBody(Vector3(-1.0f, 5.0f, 0.0f), 1.0f, 0.5f, 0.8f, 0.2f));
    engine.addObject(RigidBody(Vector3(1.0f, 8.0f, 0.0f), 1.5f, 0.5f, 0.5f, 0.2f));
    engine.addObject(RigidBody(Vector3(0.0f, 10.0f, 0.0f), 2.0f, 0.7f, 0.7f, 0.2f));
}

// -----------------------------------------------------------------------------
// Configures a basic OpenGL lighting setup. Uses a single point light source
// above the scene to highlight sphere curvature and floor shading.
// -----------------------------------------------------------------------------
void setupLighting() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    GLfloat lightPos[] = { 0.0f, 10.0f, 5.0f, 1.0f };
    GLfloat lightAmbient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat lightDiffuse[] = { 0.9f, 0.9f, 0.9f, 1.0f };

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
}

// -----------------------------------------------------------------------------
// Defines a perspective projection and places a fixed camera looking toward
// the origin where the dynamic simulation takes place.
// -----------------------------------------------------------------------------
void setupCamera() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 800.0 / 600.0, 0.1, 100.0);  // FOV, aspect ratio, near/far

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 5.0, 15.0,   // Camera position
        0.0, 0.0, 0.0,    // Target point
        0.0, 1.0, 0.0);   // Up direction
}

// -----------------------------------------------------------------------------
// Render callback: clears the display, sets camera transform again,
// draws the floor, then draws each rigid body as a shaded sphere.
// -----------------------------------------------------------------------------
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // Reapply camera transform
    gluLookAt(0.0, 5.0, 15.0,
        0.0, 0.0, 0.0,
        0.0, 1.0, 0.0);

    // ----- Draw floor plane -----
    glColor3f(0.4f, 0.4f, 0.4f);
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(-5, 0, -5);
    glVertex3f(5, 0, -5);
    glVertex3f(5, 0, 5);
    glVertex3f(-5, 0, 5);
    glEnd();

    // ----- Draw simulated rigid bodies (spheres) -----
    for (auto& obj : engine.objects) {
        glPushMatrix();
        glTranslatef(obj.position.x, obj.position.y, obj.position.z);

        // Slight color variation based on radius for visual distinction
        glColor3f(0.2f + obj.radius,
            0.3f,
            0.6f + obj.radius * 0.5f);

        glutSolidSphere(obj.radius, 24, 24);
        glPopMatrix();
    }

    glutSwapBuffers(); // Double-buffered rendering
}

// -----------------------------------------------------------------------------
// Timer callback: advances the physics simulation, requests a new frame,
// and reschedules itself to maintain ~60 FPS.
// -----------------------------------------------------------------------------
void update(int value) {
    engine.update(dt);
    glutPostRedisplay();         // Ask GLUT to redraw the scene
    glutTimerFunc(16, update, 0); // Schedule next update in ~16 ms
}

// -----------------------------------------------------------------------------
// Main entry point. Sets up the window, OpenGL states, lighting, camera,
// initializes the simulation scene, and enters the GLUT event loop.
// -----------------------------------------------------------------------------
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("CS6555 Assignment 3 - Physics");

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f); // Dark background for contrast

    setupLighting();
    setupCamera();
    initScene();

    glutDisplayFunc(display);
    glutTimerFunc(16, update, 0);
    glutMainLoop();

    return 0;
}
