#pragma once
// -----------------------------------------------------------------------------
// Solver.h - the SPH solid-mechanics solver.
//
// Usage:
//     Solver solver(scenario, defaultMaterials());
//     while (!solver.finished()) solver.step();
//
// Each step() advances the simulation by an automatically chosen time step
// (typically 10-100 nanoseconds). See Solver.cpp for the equations.
// -----------------------------------------------------------------------------
#include <cstdint>
#include <memory>
#include <vector>

#include "armor/Fragments.h"
#include "armor/Kernel.h"
#include "armor/Material.h"
#include "armor/NeighborGrid.h"
#include "armor/Particles.h"
#include "armor/Recorder.h"
#include "armor/Scenario.h"
#include "armor/SymTensor.h"
#include "armor/ThreadPool.h"

namespace armor {

struct Diagnostics {
    double kinetic = 0.0;  // J (per metre depth in plane strain)
    double internal = 0.0; // J
    Vec2 momentum;         // kg·m/s
    double maxSpeed = 0.0; // m/s
};

class Solver {
public:
    Solver(Scenario scenario, std::vector<Material> materials);

    void step();

    bool finished() const { return time_ >= scenario_.endTime * (1.0 - 1e-12); }
    double time() const { return time_; }
    double dt() const { return dt_; }
    std::uint64_t stepCount() const { return steps_; }
    double smoothingLength() const { return kernel_.h(); }

    std::size_t particleCount() const { return nReal_; }
    const Particles& particles() const { return particles_; }
    // Write access, for setting custom initial conditions (e.g. a velocity
    // field) before the first step. Don't add or remove particles.
    Particles& state() { return particles_; }
    const Scenario& scenario() const { return scenario_; }
    const std::vector<Material>& materials() const { return materials_; }

    Diagnostics diagnostics() const;
    // Pieces the bodies have broken into (see Fragments.h); link 1.5 dx.
    FragmentStats fragments() const {
        const std::size_t n = nReal_;
        const Particles& P = particles_;
        return findFragments({P.pos.data(), n}, {P.vel.data(), n}, {P.mass.data(), n}, {P.part.data(), n},
                             {P.failed.data(), n}, 1.5 * scenario_.dx);
    }
    std::unique_ptr<Frame> captureFrame() const;

private:
    void computeDerived(std::size_t i);
    void filterDensity();
    void addAxisGhosts();
    double stableTimeStep() const;
    void computeStrainRates(std::size_t i);
    void computeForces(std::size_t i);
    void updatePorosity(std::size_t i);
    void integrateCeramic(std::size_t i, SymTensor S, const JohnsonHolmquist2& jh);
    void kick(std::size_t i);
    void integrate(std::size_t i);

    Scenario scenario_;
    std::vector<Material> materials_;
    bool axisymmetric_ = false;
    CubicSplineKernel kernel_;
    double wAtSpacing_ = 0.0; // W(dx), for the artificial stress
    double artStressEps_ = 0.0; // effective artificial stress strength (0 = off)
    ThreadPool pool_;
    NeighborGrid grid_;

    Particles particles_;  // real particles, plus mirror "ghosts" during a step
    std::size_t nReal_ = 0;

    // Per-particle helpers, recomputed every step.
    std::vector<double> planarMass_;    // mass per unit depth / per radian (see Solver.cpp)
    std::vector<double> soundSpeed_;
    std::vector<SymTensor> artStress_;
    std::vector<double> pEff_;          // pressure used in forces and work (void-limited tension)
    std::vector<double> balsara_;       // Balsara viscosity factor, from the previous step's ∇v
    std::vector<std::uint32_t> ghostSource_; // ghost k mirrors real particle ghostSource_[k]
    // Rates of change, computed every step.
    std::vector<Vec2> acc_;
    std::vector<double> rhoDot_;
    std::vector<double> eDot_;
    std::vector<SymTensor> SDot_;
    std::vector<double> strainRate_; // effective (von Mises) strain rate, 1/s - for Johnson-Cook
    std::vector<double> filteredRho_; // scratch for the density filter

    double time_ = 0.0;
    double dt_ = 0.0;
    std::uint64_t steps_ = 0;
};

} // namespace armor
