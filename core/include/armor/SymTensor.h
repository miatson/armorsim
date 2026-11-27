#pragma once
// -----------------------------------------------------------------------------
// SymTensor.h - symmetric stress / strain tensor for 2D simulations.
//
// STRESS is force per area inside a material (unit: pascal, Pa = N/m²).
// At a point it has several components:
//     σxx, σyy  - normal stresses: pulling (+) or pushing (−) along x, along y
//     σxy       - shear stress: one layer sliding over the next
//     σzz       - the third normal stress, out of the 2D plane
//
// Even a "2D" simulation needs σzz:
//   - PLANE STRAIN (a slice of something very long in z): the material can't
//     stretch in z, so squeezing it in x/y builds up stress in z.
//   - AXISYMMETRIC (a body of revolution - rods, round plates): the third
//     direction is the HOOP (θ) direction around the axis. When a round plate
//     bulges outwards its circumference grows -> hoop stress.
// We store σzz in `zz` in both cases.
//
// The tensor is symmetric (σxy = σyx: otherwise a tiny cube would spin up
// infinitely fast), so 4 numbers are enough.
// -----------------------------------------------------------------------------
#include <cmath>

#include "Vec2.h"

namespace armor {

struct SymTensor {
    double xx = 0.0, yy = 0.0, xy = 0.0, zz = 0.0;

    constexpr SymTensor& operator+=(const SymTensor& o) {
        xx += o.xx; yy += o.yy; xy += o.xy; zz += o.zz;
        return *this;
    }
    constexpr double trace() const { return xx + yy + zz; }
    constexpr bool operator==(const SymTensor&) const = default;
};

constexpr SymTensor operator+(SymTensor a, const SymTensor& b) { return a += b; }
constexpr SymTensor operator*(const SymTensor& t, double s) { return {t.xx * s, t.yy * s, t.xy * s, t.zz * s}; }

// In-plane action on a vector (zz has no in-plane effect): traction t = σ·n
constexpr Vec2 operator*(const SymTensor& t, Vec2 v) { return {t.xx * v.x + t.xy * v.y, t.xy * v.x + t.yy * v.y}; }

// Double contraction a:b = Σ a_ij b_ij (xy appears twice: xy and yx).
constexpr double doubleDot(const SymTensor& a, const SymTensor& b) {
    return a.xx * b.xx + a.yy * b.yy + a.zz * b.zz + 2.0 * a.xy * b.xy;
}

// Deviator: the part of the tensor that changes SHAPE, with the volume-
// changing part (mean of the normal components) removed.
constexpr SymTensor deviator(const SymTensor& t) {
    const double mean = t.trace() / 3.0;
    return {t.xx - mean, t.yy - mean, t.xy, t.zz - mean};
}

// von Mises equivalent stress: one number for "how close is this to making
// the metal yield". Defined so that a simple pull test with stress σ gives
// exactly σ:   σ_vm = sqrt( 3/2 · S:S )   with S the deviatoric stress.
inline double vonMises(const SymTensor& deviatoric) {
    return std::sqrt(1.5 * doubleDot(deviatoric, deviatoric));
}

// In-plane principal stresses: rotate the axes until the shear disappears.
// tan(2θ) = 2σxy / (σxx − σyy). Returns s1, s2 and the rotation (c, s).
struct Principal2D {
    double s1, s2; // principal values
    double c, s;   // cos θ, sin θ of the first principal direction
};

inline Principal2D principal(const SymTensor& t) {
    const double theta = 0.5 * std::atan2(2.0 * t.xy, t.xx - t.yy);
    const double c = std::cos(theta), s = std::sin(theta);
    const double s1 = c * c * t.xx + 2.0 * c * s * t.xy + s * s * t.yy;
    const double s2 = s * s * t.xx - 2.0 * c * s * t.xy + c * c * t.yy;
    return {s1, s2, c, s};
}

} // namespace armor
