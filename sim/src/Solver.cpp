#include "armor/Solver.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "armor/Mat2.h"

// =============================================================================
// THE EQUATIONS
//   Stage A: elastic solids (everything below).
//   Stage B: pressure from the material's EOS p(ρ, e) - Mie-Grüneisen for
//            metals (see Material.h) - and Johnson-Cook plasticity with
//            plastic heating, applied as a radial return in integrate().
//
// A solid is a continuum governed by three conservation laws. Written for a
// particle moving with the material ("Lagrangian" form):
//
//   mass      dρ/dt = −ρ ∇·v                    (squeezing raises density)
//   momentum  dv/dt = (1/ρ) ∇·σ                 (stress differences push)
//   energy    de/dt = (1/ρ) σ : ε̇               (work done by stress)
//
// plus a MATERIAL MODEL that says how stress responds to deformation:
//
//   σ = −p I + S        p from the equation of state p(ρ)
//                       S (deviatoric stress) from Hooke's law in rate form:
//   dS/dt = 2G (ε̇ − ⅓ tr(ε̇) I) + (rotation terms)
//
// where ε̇ = ½(∇v + ∇vᵀ) is the STRAIN RATE (how fast the material stretches)
// and the rotation terms (Jaumann rate) rotate S along with the material, so
// that a body which merely spins doesn't build up fake stress.
//
// SPH turns each gradient into a sum over neighbours j (see Kernel.h):
//
//   ∇v_i   ≈ Σ_j V_j (v_j − v_i) ⊗ ∇W_ij            V_j = m_j/ρ_j (volume)
//   dv_i/dt = Σ_j m_j (σ_i/ρ_i² + σ_j/ρ_j² − Π_ij I + R_ij f_ij) · ∇W_ij
//
// The symmetric form of the momentum sum (σ_i/ρ_i² + σ_j/ρ_j²) makes the force
// of i on j exactly opposite to the force of j on i, so momentum is conserved.
//
// EXTRA TERMS every impact code needs:
//   Π_ij  artificial viscosity (Monaghan 1992). Shocks are discontinuities
//         thinner than a particle; without extra dissipation the solution
//         oscillates wildly behind them. Π only acts when particles approach:
//            μ = h (v_ij·x_ij) / (|x_ij|² + 0.01h²),    v_ij·x_ij < 0
//            Π = (−α c̄ μ + β μ²) / ρ̄
//         The energy it removes from motion is added to internal energy
//         (heating), so total energy is still conserved.
//   R_ij  artificial stress (Monaghan 2000; Gray, Monaghan & Swift 2001).
//         Plain SPH is unstable in TENSION: particles clump into pairs. A
//         small repulsive term proportional to the tensile principal stress
//         prevents it:  R = −ε σ_tension / ρ²,  f = (W(r)/W(dx))^n.
//
// KERNEL GRADIENT CORRECTION (Randles & Libersky 1996): near free surfaces a
// particle has neighbours on one side only and the plain SPH gradient is
// wrong. We measure how wrong with the matrix M = Σ V_j (x_j − x_i) ⊗ ∇W_ij
// (it should be the identity) and correct: ∇v = (Σ V_j (v_j−v_i)⊗∇W_ij) M⁻¹.
//
// AXISYMMETRIC MODE (x = axis, y = radius r). The equations in cylindrical
// coordinates gain "hoop" terms:
//   ∇·v      = ∂vx/∂x + ∂vy/∂y + vy/r          (a ring growing in radius
//                                               stretches in the hoop direction)
//   ε̇_θθ     = vy / r
//   ρ dvx/dt = (in-plane divergence of σ) + σxy / r
//   ρ dvy/dt = (in-plane divergence of σ) + (σyy − σθθ) / r
// SPH sums over the (x, r) plane use the mass PER RADIAN m/(2π r) (we call it
// the "planar mass") so that Σ (planar mass / ρ) · W ≈ 1, like in 2D.
// Near the axis, a particle's neighbourhood would be cut off at r = 0, so we
// add mirror-image GHOST particles at −r (with vy and σxy flipped) for
// everything within 2h of the axis.
//
// ENERGY-COMPATIBLE FORCES (2026-11-27). The internal energy rate is
//   m_i de_i/dt = m_i/ρ_i [ σ_i : L_i + σθθ_i v_r,i / r_i ],
//   L_i = Σ_j A_j (v_j − v_i) ⊗ ∇W_ij      (A_j = planar mass_j / ρ_j)
// and every force is DERIVED from it: the force on particle i is minus the
// coefficient of v_i in the total internal power Σ_k m_k de_k/dt. Then the
// work done by forces equals the change in internal energy exactly, so total
// energy is conserved by construction. Working it out (ring mass m = 2π r a):
//   dv_i/dt = Σ_j a_j/(ρ_i ρ_j) [σ_i + (r_j/r_i) σ_j] ∇W_ij − σθθ_i ŷ/(ρ_i r_i)
// In plane strain (all r = 1) this is the standard 1/(ρ_i ρ_j) SPH form. In
// axisymmetric mode the factor r_j/r_i is the ring-mass ratio m_j/m_i; it also
// produces the σxy/r and σyy/r hoop forces, so only σθθ is added explicitly.
// Axial momentum is then conserved exactly too (pair forces m_i a_ij are
// symmetric in i, j). Mirror ghosts use |r| (same ring as their source), which
// keeps the result exact by mirror symmetry.
// The earlier version (σ_i/ρ_i² + σ_j/ρ_j² plus explicit hoop forces) matched
// only approximately: an axisymmetric steel ball resting on an aluminium plate
// gained 23% energy in 40 us, a tungsten rod into steel 9%.
// Viscosity and artificial stress act as symmetric pair forces with weight
// a_j·r̄/r_i (r̄ = mean radius); the energy they remove is split equally.
//
// FAILURE (Stage C). Full failure when Johnson-Cook damage reaches 1 (ductile
// fracture) or the particle melts: it keeps no shear stress, and every pair
// involving it interacts like CONTACT between separate bodies (push only).
// Spall (tension beyond the spall strength, sustained) opens VOIDS instead:
// the particle stays bonded and keeps its shear strength but its pressure
// counts as max(p, 0) everywhere - forces and pressure work alike, so the
// energy books still balance (see Particles.h, FailureKind).
// So cracks open where particles fail, and fragments fly apart on their own.
// The energy-compatible rules carry over unchanged: such pairs are simply on
// the "other body" side of every sum.
//
// CONTACT: particles of DIFFERENT parts (e.g. projectile vs plate) interact
// through pressure only, and only when compressed (p > 0): they push but
// can't pull, and they slide without friction. Within one part, the full
// stress acts (the material holds together).
// =============================================================================

namespace armor {

namespace {

// Apply the kernel gradient correction G·M⁻¹ when M is well-conditioned.
// Isolated particles (fragments, Stage C) can have too few neighbours for a
// meaningful M; then we keep the uncorrected gradient.
Mat2 correctGradient(const Mat2& g, const Mat2& m) {
    const double det = m.determinant();
    if (det > 0.1 && det < 10.0) {
        return g * m.inverse();
    }
    return g;
}

SymTensor totalStress(const SymTensor& S, double p) { return {S.xx - p, S.yy - p, S.xy, S.zz - p}; }

} // namespace

Solver::Solver(Scenario scenario, std::vector<Material> materials)
    : scenario_(std::move(scenario)),
      materials_(std::move(materials)),
      axisymmetric_(scenario_.mode == GeometryMode::Axisymmetric),
      kernel_(scenario_.settings.hFactor * scenario_.dx),
      pool_(scenario_.settings.threads) {
    if (scenario_.dx <= 0.0) {
        throw std::invalid_argument("particle spacing must be positive");
    }
    if (scenario_.endTime <= 0.0) {
        throw std::invalid_argument("end time must be positive");
    }
    wAtSpacing_ = kernel_.W(scenario_.dx);
    artStressEps_ = (axisymmetric_ && !scenario_.settings.artStressInAxisymmetric) ? 0.0
                                                                                   : scenario_.settings.artStressEps;
    particles_ = buildParticles(scenario_, materials_);
    nReal_ = particles_.size();
    if (nReal_ == 0) {
        throw std::invalid_argument("the scenario contains no particles");
    }
}

// -----------------------------------------------------------------------------
// One time step.
// -----------------------------------------------------------------------------
void Solver::step() {
    if (finished()) {
        return;
    }
    const std::size_t n = nReal_;

    // 1. Per-particle quantities that neighbours will need.
    planarMass_.resize(n);
    soundSpeed_.resize(n);
    artStress_.resize(n);
    balsara_.resize(n, 1.0); // first step: full viscosity; ghosts appended later are dropped here
    pool_.parallelFor(n, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) computeDerived(i);
    });

    // 2. Mirror particles across the axis (axisymmetric only).
    if (axisymmetric_) {
        addAxisGhosts();
    }

    // 3. Who is near whom.
    grid_.build(particles_.pos, kernel_.support(), pool_);

    // 3b. Every few steps, clean up accumulated density noise.
    const int filterEvery = scenario_.settings.densityFilterInterval;
    if (filterEvery > 0 && steps_ > 0 && steps_ % static_cast<std::uint64_t>(filterEvery) == 0) {
        filterDensity();
    }

    // 4. Time step from the CFL condition.
    dt_ = std::min(stableTimeStep(), scenario_.endTime - time_);

    // 5. SEMI-IMPLICIT (SYMPLECTIC) EULER - the same idea as in the rigid-body
    //    engine, applied to the velocity <-> stress pair:
    //       v(n+1) = v(n) + dt · a(σ(n))           forces from the current stress
    //       σ(n+1) = σ(n) + dt · f(∇v(n+1))        stress from the NEW velocities
    //       x(n+1) = x(n) + dt · v(n+1)
    //    Using the OLD velocities for the stress update (plain forward Euler)
    //    pumps energy into every vibration: in testing the total energy grew
    //    ~0.3% per microsecond, even in a simple elastic bar collision.
    acc_.resize(n);
    rhoDot_.resize(n);
    eDot_.resize(n);
    SDot_.resize(n);
    strainRate_.resize(n);
    pool_.parallelFor(n, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) computeForces(i);
    });
    pool_.parallelFor(n, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) kick(i);
    });
    // Ghosts must see the new velocities of the particles they mirror.
    for (std::size_t k = 0; k < ghostSource_.size(); ++k) {
        const Vec2 v = particles_.vel[ghostSource_[k]];
        particles_.vel[n + k] = {v.x, -v.y};
    }
    pool_.parallelFor(n, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) computeStrainRates(i);
    });

    // 6. Advance everything else (ghosts are discarded; rebuilt next step).
    pool_.parallelFor(n, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) integrate(i);
    });
    particles_.resize(n);

    time_ += dt_;
    ++steps_;
}

void Solver::computeDerived(std::size_t i) {
    const Particles& P = particles_;
    const Material& mat = materials_[P.material[i]];
    planarMass_[i] = axisymmetric_ ? P.mass[i] / (2.0 * kPi * P.pos[i].y) : P.mass[i];
    soundSpeed_[i] = mat.soundSpeed(P.rho[i], P.energy[i]);

    // Artificial stress: only for tensile (positive) principal stresses.
    const Principal2D pr = principal(totalStress(P.S[i], P.pressure[i]));
    const double k = -artStressEps_ / (P.rho[i] * P.rho[i]);
    const double r1 = pr.s1 > 0.0 ? k * pr.s1 : 0.0;
    const double r2 = pr.s2 > 0.0 ? k * pr.s2 : 0.0;
    // Rotate back from principal axes to x-y.
    artStress_[i] = {pr.c * pr.c * r1 + pr.s * pr.s * r2, pr.s * pr.s * r1 + pr.c * pr.c * r2,
                     pr.c * pr.s * (r1 - r2), 0.0};
}

void Solver::addAxisGhosts() {
    const double reach = kernel_.support();
    ghostSource_.clear();
    for (std::size_t i = 0; i < nReal_; ++i) {
        if (particles_.pos[i].y >= reach) {
            continue;
        }
        ghostSource_.push_back(static_cast<std::uint32_t>(i));
        const std::size_t g = particles_.pushCopy(i);
        // Mirror image in the axis: y -> −y. Under that reflection vy and the
        // shear stress σxy change sign; everything else is unchanged.
        particles_.pos[g].y = -particles_.pos[g].y;
        particles_.vel[g].y = -particles_.vel[g].y;
        particles_.S[g].xy = -particles_.S[g].xy;
        planarMass_.push_back(planarMass_[i]);
        soundSpeed_.push_back(soundSpeed_[i]);
        balsara_.push_back(balsara_[i]);
        SymTensor r = artStress_[i];
        r.xy = -r.xy;
        artStress_.push_back(r);
    }
}

// -----------------------------------------------------------------------------
// SHEPARD DENSITY FILTER (Colagrossi & Landrini 2003)
//
// Density comes from integrating dρ/dt = −ρ∇·v. Each step's SPH estimate of
// ∇·v has a small error that depends on how disordered the particles are.
// In a long, violent plastic flow (particles shearing past each other for
// thousands of steps) these errors add up: some particles drift too dense,
// others too light - and a 5% density deficit in copper means ~7 GPa of fake
// TENSION. In the Taylor bar test this drift blew the simulation up, in both
// geometry modes, and worse at finer resolution (more steps, more drift).
//
// The filter resets each particle's density to a smoothed average of its own
// body's neighbourhood:
//
//     ρ_i ← Σ_j m_j W_ij / Σ_j (m_j/ρ_j) W_ij        (j includes i itself)
//
// The denominator normalises the kernel, so particles at free surfaces (with
// neighbours on one side only) keep their correct density. Smooth density
// fields pass through unchanged; only particle-scale noise is removed.
//
// Applied only where the material has flowed plastically (εp above a
// threshold): that's where the particles get disordered and the drift
// happens. In ordered elastic material the filter would only smear genuine,
// sharp stress concentrations (e.g. a contact zone a few particles wide).
// -----------------------------------------------------------------------------
void Solver::filterDensity() {
    Particles& P = particles_;
    const std::size_t n = nReal_;
    filteredRho_.resize(n);
    const double w0 = kernel_.W(0.0);
    const double minStrain = scenario_.settings.densityFilterMinPlasticStrain;
    pool_.parallelFor(n, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            if (P.plasticStrain[i] < minStrain || P.failed[i]) {
                filteredRho_[i] = P.rho[i]; // ordered elastic material: leave untouched
                continue;
            }
            double num = planarMass_[i] * w0;
            double den = planarMass_[i] / P.rho[i] * w0;
            for (const std::uint32_t j : grid_.neighbors(i)) {
                if (P.part[j] != P.part[i] || P.material[j] != P.material[i] || P.failed[j]) continue;
                const double w = kernel_.W(length(P.pos[i] - P.pos[j]));
                num += planarMass_[j] * w;
                den += planarMass_[j] / P.rho[j] * w;
            }
            filteredRho_[i] = num / den;
        }
    });
    pool_.parallelFor(n, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            P.rho[i] = filteredRho_[i];
            P.pressure[i] = materials_[P.material[i]].pressure(P.rho[i], P.energy[i]);
        }
    });
}

// CFL condition (Courant-Friedrichs-Lewy): information must not travel more
// than a fraction of a particle spacing per step, otherwise the explicit
// scheme blows up. The fastest signal is a sound wave riding on the flow:
//     dt = CFL · min_i  h / (c_i + |v_i|)
// Steel: c ≈ 5900 m/s, h = 0.65 mm  ->  dt ≈ 0.25 · 0.65e-3 / 5900 ≈ 28 ns.
//
// AXISYMMETRIC SAFETY NET: the hoop terms divide by the radius r. A particle
// near the axis that moves outward by δ gets hoop strain δ/r, and the
// resulting force also scales with 1/r: it behaves like a spring with natural
// frequency ω ≈ c / r. An explicit scheme is only stable if ω·dt stays small,
// so very close to the axis the relevant length is r, not h: L = min(h, k·r).
// (During development the axis row exploded; the real cause turned out to be
// the integration order - see step() - but this limit protects particles
// that deformation pushes towards the axis.)
double Solver::stableTimeStep() const {
    const double h = kernel_.h();
    const double k = scenario_.settings.axisTimeStepFactor;
    double dt = 1e30;
    for (std::size_t i = 0; i < nReal_; ++i) {
        const double signal = soundSpeed_[i] + length(particles_.vel[i]);
        const double len = axisymmetric_ ? std::min(h, k * particles_.pos[i].y) : h;
        dt = std::min(dt, len / signal);
    }
    return scenario_.settings.cfl * dt;
}

// -----------------------------------------------------------------------------
// Pass A: velocity gradient -> density rate, deviatoric stress rate, work.
// -----------------------------------------------------------------------------
void Solver::computeStrainRates(std::size_t i) {
    const Particles& P = particles_;
    const Vec2 xi = P.pos[i];
    const Vec2 vi = P.vel[i];
    const std::uint16_t partI = P.part[i];

    // Two gradients:
    //  - gradSame: same body only. Drives shear (different bodies slide past
    //    each other without friction).
    //  - gradAll: also includes OTHER bodies, so contact can squeeze a body
    //    (density and pressure rise, which produces the contact force) and
    //    release it again as the bodies rebound. But once the contact has
    //    fully unloaded (neither particle compressed, p ≤ 0), a neighbour of
    //    another body moving AWAY must not count as "my material being
    //    stretched": contact can't pull, so that stretching is fake, and it
    //    would build up huge tension (and energy) at the interface of two
    //    bodies that have bounced apart. Such neighbours are treated as
    //    co-moving.
    Mat2 gradAll, gradSame, mAll, mSame;
    for (const std::uint32_t j : grid_.neighbors(i)) {
        const Vec2 d = xi - P.pos[j];
        const double r = length(d);
        if (r <= 0.0) continue;
        const Vec2 gw = kernel_.gradW(d, r);
        const double vol = planarMass_[j] / P.rho[j];
        const Vec2 relVel = P.vel[j] - vi;
        const Mat2 dv = outer(relVel * vol, gw);
        const Mat2 dxm = outer((P.pos[j] - xi) * vol, gw);
        mAll += dxm;
        if (P.part[j] == partI && !fullyFailed(P.failed[i]) && !fullyFailed(P.failed[j])) { // bonded
            gradAll += dv;
            gradSame += dv;
            mSame += dxm;
        } else {
            // (v_j − v_i)·(x_i − x_j) > 0  <=>  approaching.
            // "i is compressed" (not "either is"): the contact force on i uses
            // max(p_i, 0), and counting j here exactly when that force can act
            // keeps the energy books balanced.
            const bool approaching = dot(relVel, d) > 0.0;
            if (approaching || P.pressure[i] > 0.0) {
                gradAll += dv;
            }
        }
    }
    // ENERGY CONSISTENCY: the density rate and the pressure work use the plain
    // (uncorrected) SPH divergence - the same discrete operator as the
    // symmetric pressure force in computeForces(). Then the work done by
    // pressure forces equals the change in stored energy exactly, and the
    // total energy is conserved. The gradient correction is applied only to
    // the shear (deviatoric) strain rate, where accuracy at free surfaces
    // matters more.
    const double hoop = axisymmetric_ ? vi.y / xi.y : 0.0; // vr / r
    // Raw (uncorrected) divergences for the pressure work, split into own body
    // and other bodies: contact can only push, so contact work uses max(p, 0),
    // exactly like the contact force (ENERGY-COMPATIBLE FORCES).
    const double rawDivSame = gradSame.trace() + hoop;
    const double rawDivOther = gradAll.trace() - gradSame.trace();
    if (scenario_.settings.gradientCorrection && scenario_.settings.correctedContinuity) {
        gradAll = correctGradient(gradAll, mAll);
    }
    const double divergence = gradAll.trace() + hoop;
    // The shear WORK S:ε̇ must also use the uncorrected gradient, for the same
    // reason. In elastic deformation a mismatch is harmless (the error is
    // stored and given back), but plastic work is dissipated for good, so a
    // mismatch accumulates: it made a plastic disc collision gain ~26% energy.
    const Mat2 rawSame = gradSame;
    if (scenario_.settings.gradientCorrection) {
        gradSame = correctGradient(gradSame, mSame);
    }
    rhoDot_[i] = -P.rho[i] * divergence;

    // Strain rate ε̇ = sym(∇v) and spin ω = antisym(∇v), same-part gradient.
    // gradSame.m01 = ∂vx/∂y, gradSame.m10 = ∂vy/∂x
    const double exx = gradSame.m00;
    const double eyy = gradSame.m11;
    const double exy = 0.5 * (gradSame.m01 + gradSame.m10);
    const double ezz = hoop; // plane strain: 0 (can't stretch in z); axisym: hoop
    const double meanRate = (exx + eyy + ezz) / 3.0;
    const double wxy = 0.5 * (gradSame.m01 - gradSame.m10);

    // Balsara factor for next step's artificial viscosity. Compression is
    // measured with gradAll so that contact between bodies counts (at first
    // touch a body's own particles are not yet compressed). Curl: the only
    // non-zero component in 2D / axisymmetry, ∂vy/∂x − ∂vx/∂y.
    {
        const double divAbs = std::abs(divergence);
        const double curlAbs = std::abs(gradSame.m10 - gradSame.m01);
        balsara_[i] = divAbs / (divAbs + curlAbs + 1e-4 * soundSpeed_[i] / kernel_.h());
    }

    // Effective (von Mises) strain rate sqrt(2/3 ε̇':ε̇') of the deviatoric
    // strain rate ε̇' - one number for "how fast is this being sheared",
    // used by Johnson-Cook rate hardening.
    {
        const double dxx = exx - meanRate, dyy = eyy - meanRate, dzz = ezz - meanRate;
        strainRate_[i] = std::sqrt(2.0 / 3.0 * (dxx * dxx + dyy * dyy + dzz * dzz + 2.0 * exy * exy));
    }

    // Hooke's law in rate form + Jaumann rotation of the existing stress:
    //   dS/dt = 2G (ε̇ − ⅓tr(ε̇) I) + W·S − S·W     (2D: only W_xy is non-zero)
    const double G = materials_[P.material[i]].shearModulus;
    const SymTensor& S = P.S[i];
    SDot_[i] = {2.0 * G * (exx - meanRate) + 2.0 * wxy * S.xy,
                2.0 * G * (eyy - meanRate) - 2.0 * wxy * S.xy,
                2.0 * G * exy + wxy * (S.yy - S.xx),
                2.0 * G * (ezz - meanRate)};

    // Work done on the particle per unit mass: (−p ∇·v + S:ε̇) / ρ, both with
    // the uncorrected operators (energy consistency, see above).
    // (Added to the viscous heating already stored by computeForces.)
    const double wxx = rawSame.m00, wyy = rawSame.m11, wxyRate = 0.5 * (rawSame.m01 + rawSame.m10);
    const double sDotEps = S.xx * wxx + S.yy * wyy + S.zz * ezz + 2.0 * S.xy * wxyRate;
    const double pWork = effectivePressure(P.pressure[i], P.failed[i]); // must match the force (sigmaI)
    eDot_[i] += (-pWork * rawDivSame - std::max(P.pressure[i], 0.0) * rawDivOther + sDotEps) / P.rho[i];
}

// -----------------------------------------------------------------------------
// Pass B: forces (acceleration) and viscous heating.
// -----------------------------------------------------------------------------
void Solver::computeForces(std::size_t i) {
    const Particles& P = particles_;
    const SolverSettings& s = scenario_.settings;
    const Vec2 xi = P.pos[i];
    const Vec2 vi = P.vel[i];
    const double rhoI = P.rho[i];
    const std::uint16_t partI = P.part[i];
    const SymTensor sigmaI = totalStress(P.S[i], effectivePressure(P.pressure[i], P.failed[i]));
    const double contactI = -std::max(P.pressure[i], 0.0);
    const double h = kernel_.h();
    // Ring radius (axisymmetric) or 1 (plane strain). See ENERGY-COMPATIBLE FORCES.
    const double rI = axisymmetric_ ? xi.y : 1.0;

    Vec2 acc;
    double viscousHeating = 0.0;
    double artStressWork = 0.0;
    for (const std::uint32_t j : grid_.neighbors(i)) {
        const Vec2 d = xi - P.pos[j];
        const double r2 = lengthSquared(d);
        if (r2 <= 0.0) continue;
        const double r = std::sqrt(r2);
        const Vec2 gw = kernel_.gradW(d, r);
        const double rhoJ = P.rho[j];
        // Bonded = same body and neither particle failed (see FAILURE).
        const bool samePart = P.part[j] == partI && !fullyFailed(P.failed[i]) && !fullyFailed(P.failed[j]);
        // Ghosts sit at −r but are the same ring as their source: use |r|.
        const double rJ = axisymmetric_ ? std::abs(P.pos[j].y) : 1.0;
        const double ringRatio = rJ / rI;            // m_j / m_i per unit planar mass
        const double pairWeight = 0.5 * (rI + rJ) / rI; // symmetric pair forces (viscosity, art. stress)

        // Stress part, already divided by ρ_i ρ_j and weighted (see header).
        SymTensor T;
        if (samePart) {
            T = (sigmaI + totalStress(P.S[j], effectivePressure(P.pressure[j], P.failed[j])) * ringRatio) *
                (1.0 / (rhoI * rhoJ));
        } else {
            const double c = (contactI - std::max(P.pressure[j], 0.0) * ringRatio) / (rhoI * rhoJ);
            T = {c, c, 0.0, 0.0};
        }

        // Artificial viscosity (approaching particles only).
        const Vec2 vij = vi - P.vel[j];
        const double vr = dot(vij, d);
        if (vr < 0.0) {
            const double mu = h * vr / (r2 + 0.01 * h * h);
            const double cBar = 0.5 * (soundSpeed_[i] + soundSpeed_[j]);
            const double rhoBar = 0.5 * (rhoI + rhoJ);
            const double fBal = s.balsaraSwitch ? std::max(s.balsaraMin, 0.5 * (balsara_[i] + balsara_[j])) : 1.0;
            const double pi = fBal * (-s.viscAlpha * cBar * mu + s.viscBeta * mu * mu) / rhoBar * pairWeight;
            T.xx -= pi;
            T.yy -= pi;
            viscousHeating += 0.5 * planarMass_[j] * pi * dot(vij, gw);
        }

        // Artificial stress against tensile clumping (within one body).
        // NOT between a particle and the axis ghosts (j ≥ nReal_): in tension
        // the term repels, and repelling your own mirror image pushes you away
        // from the axis -> the ring expands -> more tension -> more repulsion.
        // That feedback blew up the axis row in the Taylor bar test.
        if (samePart && artStressEps_ > 0.0 && j < nReal_) {
            const double f = kernel_.W(r) / wAtSpacing_;
            const double fn = std::pow(f, s.artStressN);
            const SymTensor R = (artStress_[i] + artStress_[j]) * (fn * pairWeight);
            T += R;
            // Its work must enter the energy equation like any other pair
            // force (same split as viscous heating), otherwise it creates
            // energy: +4% in a plane-strain ball impact.
            artStressWork += 0.5 * planarMass_[j] * dot(R * gw, P.vel[j] - vi);
        }

        acc += (T * gw) * planarMass_[j];
    }

    if (axisymmetric_) {
        // Only the hoop stress: the σxy/r and σyy/r terms come out of the
        // ring-mass factor in the pair sum (see ENERGY-COMPATIBLE FORCES).
        acc.y -= sigmaI.zz / (rhoI * xi.y);
    }

    acc_[i] = acc;
    eDot_[i] = viscousHeating + artStressWork; // the work term is added in computeStrainRates
}

// -----------------------------------------------------------------------------
// Time integration (semi-implicit Euler, as in the rigid-body engine:
// velocity first, then position with the new velocity).
// -----------------------------------------------------------------------------
void Solver::kick(std::size_t i) { particles_.vel[i] += acc_[i] * dt_; }

void Solver::integrate(std::size_t i) {
    Particles& P = particles_;
    const Material& mat = materials_[P.material[i]];

    P.pos[i] += P.vel[i] * dt_; // velocity was already updated in kick()
    // Density can't physically drop to zero; guard against runaway expansion.
    P.rho[i] = std::max(P.rho[i] + rhoDot_[i] * dt_, 0.05 * mat.density0);
    P.energy[i] += eDot_[i] * dt_;

    // ---- Plasticity: radial return (Wilkins 1964) ----
    // 1. Elastic TRIAL: assume the step was purely elastic.
    SymTensor S = P.S[i] + SDot_[i] * dt_;
    // 2. Compare with the yield stress. In stress space, the states the metal
    //    can carry form a cylinder (von Mises: σ_vm ≤ σ_y). If the trial
    //    stress lies outside, the excess deformation was PLASTIC.
    //    Rate for Johnson-Cook: the PLASTIC strain rate of the previous step
    //    (the measure JC fitted their constants with). The total strain rate
    //    is noisy in SPH - particle jitter gives short spikes that would add
    //    spurious rate hardening.
    const double rate = scenario_.settings.plasticRateForJohnsonCook ? P.plasticRate[i] : strainRate_[i];
    const double yield = mat.yieldStress(P.plasticStrain[i], rate, P.temperature[i]);
    const double vm = vonMises(S);
    P.plasticRate[i] = 0.0;
    double dEp = 0.0;
    if (vm > yield) {
        // 3. RETURN: scale the deviatoric stress straight back ("radially")
        //    onto the cylinder. The direction of S is kept, only its size
        //    shrinks to the yield stress.
        S = S * (yield / vm);
        // 4. The plastic strain increment that this corresponds to: the
        //    elastic trial overshot by (vm − σ_y), and in shear the stiffness
        //    is 3G (σ_vm = 3G·ε_eff for deviatoric strain), so
        //        Δεp = (vm − σ_y) / (3G)
        dEp = (vm - yield) / (3.0 * mat.shearModulus);
        P.plasticStrain[i] += dEp;
        P.plasticRate[i] = dEp / dt_;
        // 5. Plastic work per unit volume = σ_y · Δεp. ~90% becomes heat
        //    (Taylor-Quinney factor): ΔT = χ σ_y Δεp / (ρ c_v).
        //    (The work itself is already in the energy equation via S:ε̇.)
        P.temperature[i] += mat.taylorQuinney * yield * dEp / (P.rho[i] * mat.specificHeat);
    }
    P.S[i] = S;
    P.pressure[i] = mat.pressure(P.rho[i], P.energy[i]);

    // ---- Failure (Stage C) ----
    if (scenario_.settings.failure && !fullyFailed(P.failed[i])) {
        if (dEp > 0.0 && mat.failure) {
            // Triaxiality σ* = −p/σ_vm with the returned stress (σ_vm = σ_y).
            // Effective pressure: around spall voids the metal no longer feels
            // the hydrostatic tension.
            const double triax = -effectivePressure(P.pressure[i], P.failed[i]) / std::max(vonMises(S), 1.0);
            P.damage[i] += dEp / mat.fractureStrain(triax, P.plasticRate[i], P.temperature[i]);
        }
        // Spall needs SUSTAINED tension: at least the time a sound wave takes
        // to cross one smoothing length, the shortest pulse the particles can
        // resolve; shorter spikes are numerical. Measured spall strengths come
        // from pulses of tens of ns to µs.
        if (P.pressure[i] < -mat.spallStrength) {
            P.spallTime[i] += dt_;
        } else {
            P.spallTime[i] = 0.0;
        }
        const double resolvable = std::max(kernel_.h() / mat.soundSpeed(mat.density0), scenario_.settings.spallIncubationTime);
        if (P.damage[i] >= 1.0) {
            P.failed[i] = kDuctile;
        } else if (P.temperature[i] >= mat.meltTemperature()) {
            P.failed[i] = kMelted;
        } else if (P.failed[i] == kIntact && P.spallTime[i] >= resolvable) {
            P.failed[i] = kSpall; // voids: keeps shear strength, loses tension
        }
    }
    if (fullyFailed(P.failed[i])) {
        P.S[i] = {}; // no shear strength left (tension is cut in the pair sums)
    }

    if (axisymmetric_ && P.pos[i].y < 0.0) {
        // A ring can't have negative radius: reflect off the axis.
        P.pos[i].y = -P.pos[i].y;
        P.vel[i].y = -P.vel[i].y;
    }
}

Diagnostics Solver::diagnostics() const {
    Diagnostics d;
    for (std::size_t i = 0; i < nReal_; ++i) {
        const double m = particles_.mass[i];
        const Vec2 v = particles_.vel[i];
        d.kinetic += 0.5 * m * lengthSquared(v);
        d.internal += m * particles_.energy[i];
        d.momentum += v * m;
        d.maxSpeed = std::max(d.maxSpeed, length(v));
    }
    return d;
}

std::unique_ptr<Frame> Solver::captureFrame() const {
    auto f = std::make_unique<Frame>();
    const std::size_t n = nReal_;
    const Particles& P = particles_;
    const Diagnostics d = diagnostics();
    f->time = time_;
    f->kineticEnergy = d.kinetic;
    f->internalEnergy = d.internal;
    f->x.resize(n);
    f->y.resize(n);
    f->body.resize(n);
    for (auto& v : f->values) v.resize(n);

    auto put = [&](Field field, std::size_t i, double value) {
        f->values[static_cast<int>(field)][i] = static_cast<float>(value);
    };
    for (std::size_t i = 0; i < n; ++i) {
        const SymTensor& S = P.S[i];
        const double p = P.pressure[i];
        f->x[i] = static_cast<float>(P.pos[i].x);
        f->y[i] = static_cast<float>(P.pos[i].y);
        f->body[i] = P.part[i];
        put(Field::Material, i, P.material[i]);
        put(Field::Pressure, i, p);
        put(Field::VonMises, i, vonMises(S));
        put(Field::PlasticStrain, i, P.plasticStrain[i]);
        put(Field::Temperature, i, P.temperature[i]);
        // Fully failed = 1; spalled (voids) shown at least 0.5.
        const double shown = fullyFailed(P.failed[i]) ? 1.0 : std::min(P.damage[i], 1.0);
        put(Field::Damage, i, P.failed[i] == kSpall ? std::max(shown, 0.5) : shown);
        put(Field::Speed, i, length(P.vel[i]));
        put(Field::Density, i, P.rho[i]);
        put(Field::StressXX, i, S.xx - p);
        put(Field::StressYY, i, S.yy - p);
        put(Field::ShearXY, i, S.xy);
    }
    return f;
}

} // namespace armor
