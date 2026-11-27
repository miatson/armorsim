#pragma once
// -----------------------------------------------------------------------------
// Fragments.h - which pieces did a body break into? (Stage C)
//
// Two particles belong to the same piece when they are of the same body,
// neither has FULLY failed (ductile fracture / melting), and they are closer
// than `linkDistance`. Pieces are the connected groups of that relation
// (union-find over the neighbour grid). Fully failed particles are loose
// DEBRIS: they hold to nothing, so each is counted on its own.
//
// Choosing linkDistance: lattice neighbours sit at dx and dx·√2 ≈ 1.41 dx, so
// 1.5 dx joins an undamaged lattice, while a crack one failed particle wide
// leaves a gap of about 2 dx between the intact sides - not joined. Heavily
// stretched intact metal can exceed 1.5 dx and split into "pieces" that are
// really still one; treat counts of tiny pieces with care.
//
// Axisymmetric mode: a piece away from the axis is a RING in 3D, which in
// reality breaks up further around its circumference. Counts are then a lower
// bound; masses are the real (ring) masses.
// -----------------------------------------------------------------------------
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "armor/Vec2.h"

namespace armor {

struct Fragment {
    std::uint16_t body = 0;
    std::size_t particles = 0;
    double mass = 0.0;  // kg (per metre depth in plane strain)
    Vec2 centroid;      // m, mass-weighted
    Vec2 velocity;      // m/s, mass-weighted (momentum / mass)
};

struct FragmentStats {
    std::vector<Fragment> pieces; // heaviest first
    std::size_t debrisParticles = 0;
    double debrisMass = 0.0;
};

FragmentStats findFragments(std::span<const Vec2> pos, std::span<const Vec2> vel, std::span<const double> mass,
                            std::span<const std::uint16_t> body, std::span<const std::uint8_t> failed,
                            double linkDistance);

} // namespace armor
