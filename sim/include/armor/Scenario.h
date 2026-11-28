#pragma once
// -----------------------------------------------------------------------------
// Scenario.h - everything needed to set up one simulation run.
//
// COORDINATES (both modes): impacts travel along x (left to right).
//   - PlaneStrain: an x-y slice through objects infinitely long in z.
//     Good for any impact angle, but a "rod" is really an infinitely wide
//     plate, so penetration numbers are only qualitative.
//   - Axisymmetric: x is the symmetry axis, y is the RADIUS (y ≥ 0).
//     Every particle is a ring around the x axis, so a rectangle touching
//     the axis is really a solid cylinder, and a half-disc on the axis is a
//     sphere. This represents real 3D rods and round plates exactly - but only
//     for impacts straight along the axis (0° obliquity).
// -----------------------------------------------------------------------------
#include <cstdint>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "armor/Material.h"
#include "armor/Particles.h"
#include "armor/Vec2.h"

namespace armor {

enum class GeometryMode { PlaneStrain, Axisymmetric };

struct RectShape {
    Vec2 center;
    Vec2 halfSize;
    double angle = 0.0; // radians, counter-clockwise (plane strain only)
};

struct DiscShape {
    Vec2 center;
    double radius = 0.0;
};

// Any simple polygon (corners in order, either direction). Used for pointed
// projectile noses; in axisymmetric mode give only the y >= 0 half.
struct PolygonShape {
    std::vector<Vec2> points;
};

using Shape = std::variant<RectShape, DiscShape, PolygonShape>;

bool contains(const Shape& shape, Vec2 p);

struct Part {
    std::string name;
    Shape shape;
    int material = 0; // index into the material list
    Vec2 velocity;    // initial velocity, m/s
    double temperature = 293.15; // initial temperature, K
    // Parts with the same contactGroup (>= 0) are BONDED: they act as one body
    // (full stress across the joint, e.g. glued armor layers). -1 = separate
    // body that can only push against others (frictionless contact).
    int contactGroup = -1;
};

struct SolverSettings {
    double cfl = 0.25;          // time step safety factor (see Solver.cpp)
    double hFactor = 1.3;       // smoothing length h = hFactor · dx
    double viscAlpha = 1.0;     // artificial viscosity, linear term
    double viscBeta = 2.0;      // artificial viscosity, quadratic term
    // Balsara switch (Balsara 1995): scale the artificial viscosity by
    // f = |div v| / (|div v| + |curl v| + 1e-4 c/h). Shocks are compressive
    // (f ~ 1, full viscosity); plastic flow is mostly shear (f ~ 0). Without
    // it the viscosity damps real shear flow and metals deform too little
    // (Taylor bar ~9% too long).
    bool balsaraSwitch = true;
    // Lower bound on the Balsara factor: some viscosity always remains, to
    // damp particle-scale noise in shear zones.
    //  - Taylor bar at dx = 0.254 mm: 0.1 and 0.2 blew up, 0.3+ stable.
    //  - An axisymmetric runaway seen at 0.4 (axis rows heating without bound)
    //    was the old non-compatible force form, not the floor; with the
    //    energy-compatible forces 0.3 and 0.4 are both stable.
    //  - Taylor length at dx = 0.127 mm depends only weakly on it.
    double balsaraMin = 0.4;
    double artStressEps = 0.3;  // artificial stress strength (tensile instability fix)
    int artStressN = 4;         // artificial stress exponent
    // In axisymmetric mode, artificial stress turned out to be destabilising
    // near the axis: in tension it pushes particles apart, which expands the
    // innermost rings, which raises the hoop tension further - a runaway that
    // blew up the Taylor bar test. Off by default there; on in plane strain.
    bool artStressInAxisymmetric = false;
    bool gradientCorrection = true; // kernel gradient correction near free surfaces
    bool correctedContinuity = true; // also use the corrected gradient for the density rate
    // Johnson-Cook rate term: true = plastic strain rate from the previous step
    // (what the JC constants were fitted with); false = total effective strain
    // rate (noisier in SPH: particle jitter -> spurious rate hardening).
    bool plasticRateForJohnsonCook = true;
    // Shepard density filter every N steps (0 = off). Removes the slow drift
    // of density that integrating the continuity equation accumulates in long
    // large-deformation flows (see Solver.cpp).
    int densityFilterInterval = 20;
    // Only particles with at least this much plastic strain are filtered.
    // The drift comes from particle disorder in large plastic flow; filtering
    // ordered, elastic material only smears real stress concentrations (it
    // stopped an elastic disc collision from bouncing).
    double densityFilterMinPlasticStrain = 0.1;
    // Axisymmetric only: near the axis the hoop terms (÷ radius) act like a
    // stiff spring, so the time step also uses length min(h, k·radius).
    // With k = 3 this only kicks in for particles pushed much closer to the
    // axis than the starting lattice (innermost row at r = dx/2 ≈ 0.38h).
    double axisTimeStepFactor = 3.0;
    // Stage C: damage, spall and melting can fail particles.
    bool failure = true;
    // Spall fails a particle only after its tension has exceeded the spall
    // strength for this long (at least h/c, the shortest resolvable pulse).
    double spallIncubationTime = 0.0;
    // Spall voids coalesce into a full crack at this porosity: the final-
    // failure void fraction of the Tvergaard-Needleman ductile fracture
    // model, f_F ≈ 0.25 (Tvergaard & Needleman 1984).
    double coalescencePorosity = 0.25;
    // Porosity a spall void starts with (sets the first growth threshold).
    double initialPorosity = 1e-3;
    unsigned threads = 0;       // 0 = all CPU cores
};

struct Scenario {
    std::string name;
    GeometryMode mode = GeometryMode::PlaneStrain;
    std::vector<Part> parts;
    double dx = 0.5e-3;       // particle spacing, m
    double endTime = 50e-6;   // s
    int frameCount = 200;     // frames recorded for replay
    SolverSettings settings;
};

// Contact body id of part `partIndex`: its own index, or a shared id for
// bonded parts (same contactGroup).
inline std::uint16_t bodyId(const Scenario& s, std::size_t partIndex) {
    const int group = s.parts[partIndex].contactGroup;
    return static_cast<std::uint16_t>(group >= 0 ? 30000 + group : static_cast<int>(partIndex));
}

// Fill every part with particles on a square lattice of spacing dx.
// Lattice points are at ((i + ½)dx, (j + ½)dx) for ALL parts, so touching
// parts line up exactly, and in axisymmetric mode no particle sits on the
// axis (y = 0 would mean a ring of zero radius).
// Where parts overlap, the earlier part in the list wins.
Particles buildParticles(const Scenario& scenario, std::span<const Material> materials);

} // namespace armor
