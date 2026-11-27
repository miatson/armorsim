#include <doctest/doctest.h>

#include <cmath>
#include <string>

#include <armor/Solver.h>

using namespace armor;
using doctest::Approx;

namespace {

// Two identical steel rods/strips flying at each other at ±v.
// Axisymmetric: solid cylinders of radius R. Plane strain: strips 2R thick.
Scenario barImpact(GeometryMode mode, double v, double dx = 0.5e-3) {
    const double L = 0.06, R = 0.004;
    Scenario s;
    s.mode = mode;
    s.dx = dx;
    s.endTime = 25e-6;
    const double yc = mode == GeometryMode::Axisymmetric ? R / 2 : 0.0;
    const double hy = mode == GeometryMode::Axisymmetric ? R / 2 : R;
    s.parts.push_back({"left", RectShape{{-L / 2, yc}, {L / 2, hy}}, kSteelRHA, {v, 0.0}});
    s.parts.push_back({"right", RectShape{{L / 2, yc}, {L / 2, hy}}, kSteelRHA, {-v, 0.0}});
    return s;
}

// Average axial stress σxx of particles within dx of x = gauge, in part 1.
double gaugeStress(const Solver& solver, double gauge) {
    const Particles& P = solver.particles();
    double sum = 0.0;
    int count = 0;
    for (std::size_t i = 0; i < solver.particleCount(); ++i) {
        if (P.part[i] == 1 && std::abs(P.pos[i].x - gauge) < solver.scenario().dx) {
            sum += P.S[i].xx - P.pressure[i];
            ++count;
        }
    }
    return count ? sum / count : 0.0;
}

struct WaveResult {
    double speed;   // m/s, from the travel time between two gauges
    double plateau; // Pa, average stress at the first gauge behind the front
};

// The wave front gets smeared over a few particles, and the impact itself
// takes a moment to build up. Timing the front between TWO gauges (when each
// first sees half the expected stress) cancels both effects.
// The stress behind the front rings (in a real rod too: Pochhammer-Chree
// oscillations), so the plateau is averaged over 1-5 µs after arrival.
// threshold: fraction of the expected stress that counts as "the front has arrived".
WaveResult measureWave(GeometryMode mode, double expectedStress, double dx = 0.5e-3, double threshold = 0.5) {
    Solver solver(barImpact(mode, 5.0, dx), defaultMaterials());
    const double g1 = 0.015, g2 = 0.045;
    double t1 = -1.0, t2 = -1.0;
    double plateauSum = 0.0;
    int plateauCount = 0;
    while (!solver.finished() && (t2 < 0 || (t1 > 0 && solver.time() < t1 + 5e-6))) {
        solver.step();
        const double t = solver.time();
        const double s1 = gaugeStress(solver, g1);
        if (t1 < 0 && s1 < -threshold * expectedStress) t1 = t;
        if (t1 > 0 && t > t1 + 1e-6 && t < t1 + 5e-6) {
            plateauSum += s1;
            ++plateauCount;
        }
        if (t2 < 0 && gaugeStress(solver, g2) < -threshold * expectedStress) t2 = t;
    }
    REQUIRE(t1 > 0);
    REQUIRE(t2 > 0);
    REQUIRE(plateauCount > 0);
    return {(g2 - g1) / (t2 - t1), plateauSum / plateauCount};
}

} // namespace

TEST_CASE("a stress-free body at rest stays at rest (both modes)") {
    for (GeometryMode mode : {GeometryMode::PlaneStrain, GeometryMode::Axisymmetric}) {
        Scenario s;
        s.mode = mode;
        s.dx = 1e-3;
        s.endTime = 5e-6;
        s.parts.push_back({"block", RectShape{{0.0, 0.01}, {0.01, 0.01}}, kAluminium6061, {}});
        Solver solver(s, defaultMaterials());
        for (int i = 0; i < 50; ++i) solver.step();
        CHECK(solver.diagnostics().maxSpeed < 1e-9);
    }
}

TEST_CASE("axisymmetric rod impact: wave speed sqrt(E/rho) and stress rho*c*v") {
    const Material steel = defaultMaterials()[kSteelRHA];
    const double c = steel.barWaveSpeed();          // ~5135 m/s
    const double sigma = steel.density0 * c * 5.0;  // ~200 MPa
    const WaveResult r = measureWave(GeometryMode::Axisymmetric, sigma);
    const double measuredSpeed = r.speed;
    MESSAGE("rod wave speed: measured ", measuredSpeed, " m/s, theory ", c, " m/s");
    MESSAGE("rod stress: measured ", -r.plateau / 1e6, " MPa, theory ", sigma / 1e6, " MPa");
    CHECK(measuredSpeed == Approx(c).epsilon(0.10));
    CHECK(-r.plateau == Approx(sigma).epsilon(0.20));
}

TEST_CASE("plane strain strip impact: plate wave speed") {
    const Material steel = defaultMaterials()[kSteelRHA];
    const double c = steel.plateWaveSpeed();        // ~5370 m/s
    const double sigma = steel.density0 * c * 5.0;
    const WaveResult r = measureWave(GeometryMode::PlaneStrain, sigma);
    const double measuredSpeed = r.speed;
    MESSAGE("plate wave speed: measured ", measuredSpeed, " m/s, theory ", c, " m/s");
    CHECK(measuredSpeed == Approx(c).epsilon(0.10));
    CHECK(-r.plateau == Approx(sigma).epsilon(0.20));
}

// PLATE IMPACT (uniaxial strain) - the standard hydrocode validation test.
// Two wide plates collide face-on. Near the centre line the material can't
// expand sideways before edge effects arrive, so the wave is purely 1D and
// does NOT disperse. Exact theory:
//     wave speed   cL = sqrt((K + 4G/3) / ρ)
//     stress       σ  = ρ · cL · v     (each plate at ±v)
// (The rod/strip tests above have ±5-10% tolerances because a wave in a
// finite-width bar really does disperse: its front spreads as it travels.)
TEST_CASE("plate impact: exact longitudinal wave speed and stress (both modes)") {
    const Material steel = defaultMaterials()[kSteelRHA];
    const double cL = steel.soundSpeed(steel.density0);
    const double expected = steel.density0 * cL * 5.0;
    for (GeometryMode mode : {GeometryMode::PlaneStrain, GeometryMode::Axisymmetric}) {
        const bool axi = mode == GeometryMode::Axisymmetric;
        const double dx = 0.5e-3;
        Scenario s;
        s.mode = mode;
        s.dx = dx;
        s.endTime = 12e-6;
        // Plane strain: plates 100 mm tall centred on y=0. Axisymmetric: discs of radius 50 mm.
        const double yc = axi ? 0.025 : 0.0, hy = axi ? 0.025 : 0.05;
        s.parts.push_back({"flyer", RectShape{{-0.01, yc}, {0.01, hy}}, kSteelRHA, {5.0, 0.0}});
        s.parts.push_back({"target", RectShape{{0.03, yc}, {0.03, hy}}, kSteelRHA, {-5.0, 0.0}});
        Solver solver(s, defaultMaterials());

        auto gauge = [&](double gx) { // centre-line average of σxx in the target
            const Particles& P = solver.particles();
            double sum = 0.0;
            int n = 0;
            for (std::size_t i = 0; i < solver.particleCount(); ++i) {
                if (P.part[i] == 1 && std::abs(P.pos[i].x - gx) < dx && std::abs(P.pos[i].y) < 0.005) {
                    sum += P.S[i].xx - P.pressure[i];
                    ++n;
                }
            }
            return n ? sum / n : 0.0;
        };
        double t1 = -1.0, t2 = -1.0;
        while (!solver.finished() && t2 < 0) {
            solver.step();
            if (t1 < 0 && gauge(0.01) < -0.5 * expected) t1 = solver.time();
            if (t2 < 0 && gauge(0.04) < -0.5 * expected) t2 = solver.time();
        }
        REQUIRE(t1 > 0);
        REQUIRE(t2 > 0);
        const double speed = 0.03 / (t2 - t1);
        const double stress = -gauge(0.01);
        const std::string label = axi ? "axisymmetric" : "plane strain";
        MESSAGE(label, " plate impact: speed ", speed, " m/s (theory ", cL,
                "), stress ", stress / 1e6, " MPa (theory ", expected / 1e6, ")");
        CHECK(speed == Approx(cL).epsilon(0.02));
        CHECK(stress == Approx(expected).epsilon(0.03));
    }
}

TEST_CASE("total energy and momentum are conserved in an elastic collision") {
    Scenario s;
    s.mode = GeometryMode::PlaneStrain;
    s.dx = 0.5e-3;
    s.endTime = 30e-6;
    s.parts.push_back({"a", DiscShape{{-0.0055, 0.0}, 0.005}, kAluminium6061, {20.0, 0.0}});
    s.parts.push_back({"b", DiscShape{{0.0055, 0.0}, 0.005}, kAluminium6061, {-20.0, 0.0}});
    // Purely elastic version of the material: the Hertz contact stress at
    // 20 m/s would otherwise exceed aluminium's yield stress (Stage B).
    std::vector<Material> mats = defaultMaterials();
    mats[kAluminium6061].strength = ElasticStrength{};
    Solver solver(s, mats);
    const Diagnostics d0 = solver.diagnostics();
    while (!solver.finished()) solver.step();
    const Diagnostics d1 = solver.diagnostics();
    const double e0 = d0.kinetic + d0.internal;
    const double e1 = d1.kinetic + d1.internal;
    MESSAGE("energy before ", e0, " J/m, after ", e1, " J/m");
    CHECK(e1 == Approx(e0).epsilon(0.05));
    CHECK(std::abs(d1.momentum.x) < 1e-6 * d0.kinetic);
}

TEST_CASE("long bar collision incl. separation conserves energy (both modes)") {
    for (GeometryMode mode : {GeometryMode::Axisymmetric, GeometryMode::PlaneStrain}) {
        const bool axi = mode == GeometryMode::Axisymmetric;
        Scenario s;
        s.mode = mode;
        s.dx = 0.5e-3;
        s.endTime = 60e-6; // bars separate at ~39 µs
        const double yc = axi ? 0.0025 : 0.0, hy = axi ? 0.0025 : 0.005;
        s.parts.push_back({"left", RectShape{{-0.05, yc}, {0.05, hy}}, kSteelRHA, {5.0, 0.0}});
        s.parts.push_back({"right", RectShape{{0.05, yc}, {0.05, hy}}, kSteelRHA, {-5.0, 0.0}});
        Solver solver(s, defaultMaterials());
        const Diagnostics d0 = solver.diagnostics();
        while (!solver.finished() && solver.diagnostics().maxSpeed < 100.0) solver.step();
        const Diagnostics d1 = solver.diagnostics();
        const double ratio = (d1.kinetic + d1.internal) / (d0.kinetic + d0.internal);
        const std::string label = axi ? "axisymmetric" : "plane strain";
        MESSAGE(label, ": energy ratio ", ratio, ", max speed ", d1.maxSpeed, " m/s, kinetic share ",
                d1.kinetic / (d1.kinetic + d1.internal));
        CHECK(solver.finished());
        CHECK(d1.maxSpeed < 10.0);
        CHECK(ratio == Approx(1.0).epsilon(0.05).scale(0)); // scale(0): a true ±5%
    }
}

TEST_CASE("results are deterministic (bit-identical between runs)") {
    auto run = [] {
        Scenario s = barImpact(GeometryMode::Axisymmetric, 5.0);
        s.endTime = 3e-6;
        Solver solver(s, defaultMaterials());
        while (!solver.finished()) solver.step();
        return solver.particles();
    };
    const Particles a = run();
    const Particles b = run();
    REQUIRE(a.size() == b.size());
    bool same = true;
    for (std::size_t i = 0; i < a.size(); ++i) {
        same = same && a.pos[i] == b.pos[i] && a.vel[i] == b.vel[i] && a.S[i] == b.S[i];
    }
    CHECK(same);
}
