#pragma once
#include <cmath>

/**
 * @struct Vector3
 *
 * @brief A lightweight 3D vector class supporting common arithmetic and
 *        geometric operations.
 *
 * Used throughout the physics engine to represent positions, velocities,
 * forces, torques, and directions. The class provides operator overloads
 * for intuitive mathematical expressions and includes dot/cross products,
 * normalization, and scalar arithmetic.
 */
struct Vector3 {
    float x, y, z;

    /**
     * @brief Constructs a 3D vector with given components.
     * @param X Initial x component
     * @param Y Initial y component
     * @param Z Initial z component
     */
    Vector3(float X = 0, float Y = 0, float Z = 0) : x(X), y(Y), z(Z) {}

    // -------------------------
    // Arithmetic Operators
    // -------------------------

    /// Component-wise addition
    Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }

    /// Component-wise subtraction
    Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }

    /// Scalar multiplication
    Vector3 operator*(float s) const { return Vector3(x * s, y * s, z * s); }

    /// Scalar division (no zero check for performance)
    Vector3 operator/(float s) const { return Vector3(x / s, y / s, z / s); }

    /// Unary negation (used for reversing torque direction, etc.)
    Vector3 operator-() const { return Vector3(-x, -y, -z); }

    // -------------------------
    // Compound Assignment
    // -------------------------

    /// Adds another vector to this one
    Vector3& operator+=(const Vector3& v) { x += v.x; y += v.y; z += v.z; return *this; }

    /// Subtracts another vector from this one
    Vector3& operator-=(const Vector3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }

    /// Multiplies each component by a scalar
    Vector3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    /// Divides each component by a scalar
    Vector3& operator/=(float s) { x /= s; y /= s; z /= s; return *this; }

    // -------------------------
    // Vector Operations
    // -------------------------

    /**
     * @brief Returns the Euclidean length (magnitude) of the vector.
     */
    float length() const { return std::sqrt(x * x + y * y + z * z); }

    /**
     * @brief Returns a unit vector pointing in the same direction.
     *
     * If the vector has near-zero length, returns a safe default (0,1,0)
     * instead of producing NaNs.
     */
    Vector3 normalized() const {
        float len = length();
        return (len > 0) ? (*this) / len : Vector3(0, 1, 0);
    }

    /**
     * @brief Computes the dot product of this vector with another.
     * @return Scalar projection magnitude.
     */
    float dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; }

    /**
     * @brief Computes the cross product (this × v).
     * @return A vector perpendicular to both inputs following the right-hand rule.
     */
    Vector3 cross(const Vector3& v) const {
        return Vector3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }
};

/**
 * @brief Allows scalar * vector multiplication in either order.
 * Example: Vector3 v = 3.0f * Vector3(1, 0, 0);
 */
inline Vector3 operator*(float s, const Vector3& v) { return v * s; }
