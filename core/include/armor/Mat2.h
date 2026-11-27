#pragma once
// -----------------------------------------------------------------------------
// Mat2.h - general 2x2 matrix (double).
//
//     | m00 m01 |
//     | m10 m11 |
//
// In the solver it mostly holds a VELOCITY GRADIENT:
//     | ∂vx/∂x  ∂vx/∂y |
//     | ∂vy/∂x  ∂vy/∂y |
// which tells how the velocity changes as you move across the material.
// Its symmetric part is the strain rate (stretching), its antisymmetric part
// is the spin (rotation).
// -----------------------------------------------------------------------------
#include "Vec2.h"

namespace armor {

struct Mat2 {
    double m00 = 0.0, m01 = 0.0;
    double m10 = 0.0, m11 = 0.0;

    static constexpr Mat2 identity() { return {1.0, 0.0, 0.0, 1.0}; }

    constexpr double determinant() const { return m00 * m11 - m01 * m10; }
    constexpr double trace() const { return m00 + m11; }

    // |a b|⁻¹ = 1/(ad-bc) · | d -b|
    // |c d|                 |-c  a|     Caller must check the determinant first.
    constexpr Mat2 inverse() const {
        const double inv = 1.0 / determinant();
        return {m11 * inv, -m01 * inv, -m10 * inv, m00 * inv};
    }

    constexpr Mat2& operator+=(const Mat2& o) {
        m00 += o.m00; m01 += o.m01; m10 += o.m10; m11 += o.m11;
        return *this;
    }
};

constexpr Mat2 operator*(const Mat2& a, const Mat2& b) {
    return {a.m00 * b.m00 + a.m01 * b.m10, a.m00 * b.m01 + a.m01 * b.m11,
            a.m10 * b.m00 + a.m11 * b.m10, a.m10 * b.m01 + a.m11 * b.m11};
}

constexpr Vec2 operator*(const Mat2& m, Vec2 v) {
    return {m.m00 * v.x + m.m01 * v.y, m.m10 * v.x + m.m11 * v.y};
}

// Outer (tensor) product a ⊗ b: the matrix with entries a_i · b_j.
constexpr Mat2 outer(Vec2 a, Vec2 b) { return {a.x * b.x, a.x * b.y, a.y * b.x, a.y * b.y}; }

} // namespace armor
