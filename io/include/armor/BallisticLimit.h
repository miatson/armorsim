#pragma once
// -----------------------------------------------------------------------------
// BallisticLimit.h - the V50 / ballistic-limit tool (Stage E).
//
// The ballistic limit is the impact speed at which a projectile just gets
// through a plate. Experiments bracket it with shots that stop and shots that
// perforate; we do the same with simulations:
//   fireShot()            one simulated shot -> perforated? residual velocity
//   findBallisticLimit()  bracket [stopped, perforated], then halve it
// plus the Recht-Ipson fit through the perforating shots (residual velocity
// curve with exponent p = 2):  v_r = a · sqrt(v² − v_bl²).
//
// PERFORATED means: the projectile has passed the CURRENT back surface of the
// plate (the largest target piece, measured across the projectile's width) and
// moves faster than that piece. The plate is not clamped in the simulation, so
// "faster than the plate" (not just "moving") matters near the limit, where a
// stopped projectile otherwise rides along with a slowly moving plate.
// The projectile is part 0 of the scenario; everything else is target.
// -----------------------------------------------------------------------------
#include <functional>
#include <span>
#include <vector>

#include <armor/Material.h>
#include <armor/Scenario.h>

namespace armor {

struct ShotResult {
    double speed = 0.0;            // impact speed, m/s
    bool perforated = false;
    double residualVelocity = 0.0; // projectile's mass-weighted x velocity at the end, m/s
    double plateVelocity = 0.0;    // largest target piece's x velocity at the end, m/s
    double timeSimulated = 0.0;    // s (shots end early once the outcome is clear)
    double wallSeconds = 0.0;
    bool blewUp = false;
};

ShotResult fireShot(const Scenario& scenario, std::span<const Material> materials);

struct BallisticLimitResult {
    std::vector<ShotResult> shots; // in the order fired
    double highestStopped = 0.0;   // m/s (0 if none)
    double lowestPerforated = 0.0; // m/s (0 if none)
    bool bracketed() const { return highestStopped > 0.0 && lowestPerforated > 0.0; }
    double v50() const { return 0.5 * (highestStopped + lowestPerforated); }
    // Recht-Ipson fit through the perforating shots (needs >= 2 of them).
    bool hasFit = false;
    double fitLimit = 0.0; // v_bl, m/s
    double fitA = 0.0;     // a (momentum share: projectile / (projectile + plug))
};

// makeScenario(v) must return the scenario for impact speed v. Fires `low`
// and `high` (widening outward up to 4 times if they don't bracket), then
// halves the bracket `iterations` times. onShot is called after every shot.
BallisticLimitResult findBallisticLimit(const std::function<Scenario(double)>& makeScenario, double low, double high,
                                        int iterations, std::span<const Material> materials,
                                        const std::function<void(const ShotResult&)>& onShot = {});

// Recht-Ipson fit (p = 2) through shots with v > 0 residual: least squares on
// v_r² = a² v² − a² v_bl². Returns false if fewer than 2 usable shots.
bool rechtIpsonFit(std::span<const ShotResult> shots, double& limit, double& a);

} // namespace armor
