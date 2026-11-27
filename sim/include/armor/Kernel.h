#pragma once
// -----------------------------------------------------------------------------
// Kernel.h - the SPH smoothing kernel.
//
// SPH ("Smoothed Particle Hydrodynamics") represents a material as particles.
// Any field A (density, velocity, stress...) at a point x is estimated by a
// weighted average of the particles around it:
//
//     A(x) ≈ Σ_j  (m_j / ρ_j) · A_j · W(|x − x_j|, h)
//
//   m_j / ρ_j = the particle's volume (area in 2D)
//   W         = the KERNEL: a smooth bell-shaped weight that is largest at
//               distance 0 and falls to zero at distance 2h
//   h         = smoothing length (how far each particle "reaches")
//
// The magic of SPH: GRADIENTS (how fast something changes in space) only
// need the gradient of W, which we know analytically:
//
//     ∇A(x) ≈ Σ_j (m_j / ρ_j) · A_j · ∇W(x − x_j, h)
//
// so stress gradients -> forces, velocity gradients -> strain rates, all
// without any mesh. Particles can separate freely, which is why SPH is used
// for fragmenting impacts.
//
// We use the classic CUBIC SPLINE kernel (Monaghan & Lattanzio 1985). In 2D,
// with q = r/h:
//
//            σ · (1 − 1.5 q² + 0.75 q³)      0 ≤ q < 1
//   W(r) =   σ · 0.25 · (2 − q)³              1 ≤ q < 2
//            0                                q ≥ 2
//
// with σ = 10 / (7π h²), chosen so that the integral of W over the plane is
// exactly 1 (a weighted average must have weights summing to 1).
// -----------------------------------------------------------------------------
#include "armor/Vec2.h"

namespace armor {

inline constexpr double kPi = 3.14159265358979323846;

class CubicSplineKernel {
public:
    explicit CubicSplineKernel(double h = 1.0) : h_(h), sigma_(10.0 / (7.0 * kPi * h * h)) {}

    double h() const { return h_; }
    double support() const { return 2.0 * h_; } // particles further than this don't interact

    double W(double r) const {
        const double q = r / h_;
        if (q < 1.0) {
            return sigma_ * (1.0 - 1.5 * q * q + 0.75 * q * q * q);
        }
        if (q < 2.0) {
            const double t = 2.0 - q;
            return sigma_ * 0.25 * t * t * t;
        }
        return 0.0;
    }

    // dW/dr (always ≤ 0: the weight decreases with distance).
    double dWdr(double r) const {
        const double q = r / h_;
        if (q < 1.0) {
            return sigma_ / h_ * (-3.0 * q + 2.25 * q * q);
        }
        if (q < 2.0) {
            const double t = 2.0 - q;
            return sigma_ / h_ * (-0.75 * t * t);
        }
        return 0.0;
    }

    // ∇_i W(x_i − x_j): d = x_i − x_j, r = |d|. Chain rule: ∇W = dW/dr · d/r.
    Vec2 gradW(Vec2 d, double r) const {
        if (r <= 0.0) {
            return {};
        }
        return d * (dWdr(r) / r);
    }

private:
    double h_;
    double sigma_;
};

} // namespace armor
