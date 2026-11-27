// Stage B tests: Mie-Grüneisen EOS, Johnson-Cook strength, plasticity, and
// two classic validation experiments (shock Hugoniot, Taylor bar).
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <tuple>

#include <armor/Solver.h>

using namespace armor;
using doctest::Approx;

// -----------------------------------------------------------------------------
// Equation of state
// -----------------------------------------------------------------------------

TEST_CASE("Mie-Gruneisen pressure lands exactly on the measured shock Hugoniot") {
    // A shock with particle velocity up travels at Us = c0 + s·up and
    // produces pressure ρ0·Us·up (momentum conservation) at density
    // ρ = ρ0·Us/(Us − up) (mass conservation) and energy e = up²/2.
    const Material cu = defaultMaterials()[kCopperOFHC];
    const auto& mg = std::get<MieGruneisenEos>(cu.eos);
    for (double up : {100.0, 500.0, 1000.0, 2000.0}) {
        CAPTURE(up);
        const double us = mg.c0 + mg.s * up;
        const double rho = cu.density0 * us / (us - up);
        const double e = 0.5 * up * up;
        CHECK(cu.pressure(rho, e) == Approx(cu.density0 * us * up).epsilon(1e-9));
    }
}

TEST_CASE("Mie-Gruneisen: small strains match the linear spring, large ones are stiffer") {
    const Material steel = defaultMaterials()[kSteelRHA];
    const double K = steel.bulkModulus;
    const double small = steel.density0 * 1.001;
    CHECK(steel.pressure(small, 0.0) == Approx(K * 0.001).epsilon(0.01));
    // On the Hugoniot, 25% denser: the linear law gives 0.25 K; Mie-Grüneisen
    // gives ~0.37 K - metals stiffen a lot under strong shock compression.
    const double us = 4578.0 / (1.0 - 1.33 * 0.2); // η = 0.2 -> ρ = 1.25 ρ0
    const double up = 0.2 * us;
    CHECK(steel.pressure(steel.density0 * 1.25, 0.5 * up * up) > 1.4 * K * 0.25);
    const double big = steel.density0 * 1.1;
    // Heating at constant density raises the pressure (Grüneisen).
    CHECK(steel.pressure(big, 1e5) > steel.pressure(big, 0.0));
    // Sound speed rises under compression.
    CHECK(steel.bulkSoundSpeed(big, 0.0) > steel.bulkSoundSpeed(steel.density0, 0.0));
}

// -----------------------------------------------------------------------------
// Johnson-Cook
// -----------------------------------------------------------------------------

TEST_CASE("Johnson-Cook yield stress: hardening, rate and temperature") {
    const Material steel = defaultMaterials()[kSteelRHA];
    const auto& jc = std::get<JohnsonCookStrength>(steel.strength);
    const double T0 = kRoomTemperature;
    CHECK(steel.yieldStress(0.0, 1.0, T0) == Approx(jc.A));                                      // initial yield
    CHECK(steel.yieldStress(0.2, 1.0, T0) == Approx(jc.A + jc.B * std::pow(0.2, jc.n)));          // work hardening
    CHECK(steel.yieldStress(0.0, 1000.0, T0) == Approx(jc.A * (1 + jc.C * std::log(1000.0))));    // rate
    CHECK(steel.yieldStress(0.0, 1e-3, T0) == Approx(jc.A));                                      // slow: no softening below ref rate
    CHECK(steel.yieldStress(0.0, 1.0, jc.meltTemperature) == Approx(0.0));                        // molten
    CHECK(steel.yieldStress(0.0, 1.0, 900.0) < jc.A);                                             // hot = softer
    // Elastic materials never yield.
    CHECK(std::isinf(Material::fromYoungPoisson("x", 1000, 1e9, 0.3, "").yieldStress(1, 1, 300)));
}

// -----------------------------------------------------------------------------
// Plasticity inside the solver: a block sheared hard must cap its von Mises
// stress at the yield stress and accumulate plastic strain and heat.
// -----------------------------------------------------------------------------
TEST_CASE("radial return caps von Mises stress at the yield stress") {
    Scenario s;
    s.mode = GeometryMode::PlaneStrain;
    s.dx = 0.5e-3;
    s.endTime = 4e-6;
    s.parts.push_back({"block", RectShape{{0.0, 0.0}, {0.01, 0.01}}, kAluminium6061, {}});
    Solver solver(s, defaultMaterials());
    // Impose a strong shear flow v_x = γ̇·y (γ̇ = 2e4 /s).
    Particles& P = solver.state();
    for (std::size_t i = 0; i < solver.particleCount(); ++i) P.vel[i] = {2e4 * P.pos[i].y, 0.0};
    while (!solver.finished()) solver.step();

    const Material al = defaultMaterials()[kAluminium6061];
    const Particles& Q = solver.particles();
    double maxRatio = 0.0, maxEp = 0.0, maxT = 0.0;
    for (std::size_t i = 0; i < solver.particleCount(); ++i) {
        const double y = al.yieldStress(Q.plasticStrain[i], 2e4, Q.temperature[i]);
        maxRatio = std::max(maxRatio, vonMises(Q.S[i]) / y);
        maxEp = std::max(maxEp, Q.plasticStrain[i]);
        maxT = std::max(maxT, Q.temperature[i]);
    }
    MESSAGE("max vm/yield ", maxRatio, ", max plastic strain ", maxEp, ", max temperature ", maxT, " K");
    CHECK(maxRatio <= 1.05);  // never meaningfully above yield
    CHECK(maxEp > 0.01);      // it did flow plastically
    CHECK(maxT > kRoomTemperature + 1.0); // and heated up
}

// -----------------------------------------------------------------------------
// VALIDATION 1 - shock pressure (plate impact, copper on copper).
// Two copper plates at ±v: the interface stops, so the particle velocity
// behind each shock is up = v, and the shock pressure must follow the
// measured Hugoniot p = ρ0 (c0 + s·up) up.  At v = 500 m/s: ~21 GPa.
// -----------------------------------------------------------------------------
TEST_CASE("plate impact shock pressure matches the copper Hugoniot") {
    const Material cu = defaultMaterials()[kCopperOFHC];
    const auto& mg = std::get<MieGruneisenEos>(cu.eos);
    const double v = 500.0;
    const double expected = cu.density0 * (mg.c0 + mg.s * v) * v;

    Scenario s;
    s.mode = GeometryMode::Axisymmetric;
    s.dx = 0.25e-3;
    s.endTime = 1.5e-6; // before release waves from the flyer's back face reach the gauge
    s.parts.push_back({"flyer", RectShape{{-0.0025, 0.0125}, {0.0025, 0.0125}}, kCopperOFHC, {v, 0.0}});
    s.parts.push_back({"target", RectShape{{0.005, 0.0125}, {0.005, 0.0125}}, kCopperOFHC, {-v, 0.0}});
    Solver solver(s, defaultMaterials());
    // Guard: a numerical blow-up must fail the test, not hang it.
    while (!solver.finished() && solver.diagnostics().maxSpeed < 5 * v) solver.step();
    REQUIRE(solver.finished());

    const Particles& P = solver.particles();
    double p = 0.0, sxx = 0.0;
    int n = 0;
    for (std::size_t i = 0; i < solver.particleCount(); ++i) {
        if (P.part[i] == 1 && std::abs(P.pos[i].x - 0.003) < s.dx && P.pos[i].y < 0.004) {
            p += P.pressure[i];
            sxx += P.S[i].xx - P.pressure[i];
            ++n;
        }
    }
    REQUIRE(n > 0);
    p /= n;
    sxx /= n;
    MESSAGE("shock pressure ", p / 1e9, " GPa (Hugoniot ", expected / 1e9, " GPa), longitudinal stress ",
            -sxx / 1e9, " GPa");
    CHECK(p == Approx(expected).epsilon(0.05));
}

// -----------------------------------------------------------------------------
// VALIDATION 2 - TAYLOR BAR TEST (Taylor 1948; Johnson & Cook 1983;
// Johnson & Holmquist 1988). An OFHC copper cylinder, 25.4 mm long and
// 7.62 mm in diameter, hits a rigid wall at 190 m/s. It mushrooms at the
// impact end and shortens. Measured final length: 16.2 mm.
//
// A rod hitting a RIGID FRICTIONLESS WALL is equivalent to two identical rods
// hitting each other at ±190 m/s: by symmetry the plane between them never
// moves, which is exactly what a rigid wall enforces.
// -----------------------------------------------------------------------------
TEST_CASE("Taylor bar test: OFHC copper at 190 m/s shortens to the measured length") {
    const double L = 25.4e-3, R = 3.81e-3, v = 190.0;
    Scenario s;
    s.mode = GeometryMode::Axisymmetric;
    // 30 particles across the radius, 200 along the rod. This is the converged
    // resolution: at dx = 0.254 mm the rod comes out ~0.4 mm longer (17.0 mm),
    // at 0.127 mm the length barely depends on the viscosity settings.
    s.dx = 0.127e-3;
    s.endTime = 80e-6;
    // Failure (Stage C) is ON: release waves from the side surface focus on
    // the axis and pull the core to about -4.4 GPa for ~0.7 us (converged in
    // dx), which opens spall voids in the centre 1-5 mm behind the impact face.
    // Real annealed OFHC rods show exactly that: "ductile void damage within
    // the central region ... approximately 3 mm from the impact interface" at
    // 175 m/s (Gray, Maudlin & Chen, LANL LA-UR-01-5614).
    s.parts.push_back({"mirror rod", RectShape{{-L / 2, R / 2}, {L / 2, R / 2}}, kCopperOFHC, {v, 0.0}});
    s.parts.push_back({"rod", RectShape{{L / 2, R / 2}, {L / 2, R / 2}}, kCopperOFHC, {-v, 0.0}});
    Solver solver(s, defaultMaterials());
    const Diagnostics d0 = solver.diagnostics();
    // Guard: a numerical blow-up must fail the test, not hang it.
    while (!solver.finished() && solver.diagnostics().maxSpeed < 2 * v) solver.step();
    REQUIRE(solver.finished());
    const Diagnostics d1 = solver.diagnostics();
    MESSAGE("Taylor bar energy ratio ", (d1.kinetic + d1.internal) / (d0.kinetic + d0.internal));

    const Particles& P = solver.particles();
    // Where are the spall voids? (Particles keep their order, so the starting
    // positions tell where each one came from.)
    {
        Solver fresh(s, defaultMaterials());
        const Particles& P0 = fresh.particles();
        int nVoid = 0, nFull = 0;
        double sumX = 0.0, sumR = 0.0;
        for (std::size_t i = 0; i < solver.particleCount(); ++i) {
            if (P.part[i] != 1) continue;
            if (P.failed[i] == kSpall) {
                ++nVoid;
                sumX += P0.pos[i].x;
                sumR += P0.pos[i].y;
            } else if (fullyFailed(P.failed[i])) {
                ++nFull;
            }
        }
        MESSAGE("Taylor bar spall voids: ", nVoid, " particles, mean start x ", nVoid ? sumX / nVoid * 1e3 : 0.0,
                " mm from the face, mean start r ", nVoid ? sumR / nVoid * 1e3 : 0.0, " mm; fully failed: ", nFull);
        CHECK(nVoid > 20);                // voids form...
        CHECK(sumX / nVoid > 1.0e-3);     // ...in the central region, a few mm
        CHECK(sumX / nVoid < 4.5e-3);     //    behind the impact face (Gray et al.: ~3 mm)
        CHECK(sumR / nVoid < 1.5e-3);     //    near the axis
        CHECK(nFull < nVoid / 10);        // and the rod does not break apart
    }
    double maxX = -1.0, minX = 1.0, maxR = 0.0, maxEp = 0.0, maxT = 0.0;
    for (std::size_t i = 0; i < solver.particleCount(); ++i) {
        if (P.part[i] != 1) continue;
        maxX = std::max(maxX, P.pos[i].x);
        minX = std::min(minX, P.pos[i].x);
        maxR = std::max(maxR, P.pos[i].y);
        maxEp = std::max(maxEp, P.plasticStrain[i]);
        maxT = std::max(maxT, P.temperature[i]);
    }
    // Particle centres sit half a spacing inside the surface. The "wall" is the
    // symmetry plane x = 0, so the rod length is measured from there.
    MESSAGE("Taylor bar extents: min x ", minX * 1e3, " mm, max x ", maxX * 1e3, " mm");
    const double length = maxX + 0.5 * s.dx;
    const double footDiameter = 2.0 * (maxR + 0.5 * s.dx);
    MESSAGE("Taylor bar: final length ", length * 1e3, " mm (experiment 16.2 mm), foot diameter ",
            footDiameter * 1e3, " mm (experiment ~13.5 mm), max plastic strain ", maxEp, ", max temperature ",
            maxT, " K, rod speed now ", solver.diagnostics().maxSpeed, " m/s");
    // .scale(0) makes this a true ±5% (doctest's default adds an absolute
    // tolerance that would accept anything within 5 cm here).
    CHECK(length == Approx(16.2e-3).epsilon(0.05).scale(0));
    CHECK(footDiameter > 11.0e-3);
    CHECK(footDiameter < 15.5e-3);
}

// -----------------------------------------------------------------------------
// ENERGY IN CONTACT (regression). An axisymmetric steel ball pressing into an
// aluminium plate, and the same impact as a plane-strain slice at 500 m/s with
// plastic flow. Before the energy-compatible force form (Solver.cpp header),
// the slow axisymmetric case GAINED 23% energy in 40 us: rings of different
// radius carry different mass, and the old forces ignored that.
// Total energy may only drop slightly (time integration dissipates a little);
// it must never grow.
// -----------------------------------------------------------------------------
TEST_CASE("ball on plate conserves energy in contact (axisymmetric and plane strain)") {
    auto run = [](GeometryMode mode, double v, double dx, double endTime) {
        Scenario s;
        s.mode = mode;
        s.dx = dx;
        s.endTime = endTime;
        const bool axi = mode == GeometryMode::Axisymmetric;
        s.parts.push_back({"ball", DiscShape{{-0.0051, 0.0}, 0.005}, kSteelRHA, {v, 0.0}});
        s.parts.push_back({"plate", RectShape{{0.005, axi ? 0.02 : 0.0}, {0.005, 0.02}}, kAluminium6061, {}});
        Solver solver(s, defaultMaterials());
        const Diagnostics d0 = solver.diagnostics();
        double worst = 1.0;
        while (!solver.finished() && solver.diagnostics().maxSpeed < 5 * v) {
            solver.step();
            const Diagnostics d = solver.diagnostics();
            worst = std::max(worst, (d.kinetic + d.internal) / (d0.kinetic + d0.internal));
        }
        REQUIRE(solver.finished());
        const Diagnostics d1 = solver.diagnostics();
        const double ratio = (d1.kinetic + d1.internal) / (d0.kinetic + d0.internal);
        MESSAGE((axi ? "axisymmetric" : "plane strain"), " ball at ", v, " m/s: final energy ratio ", ratio,
                ", highest ", worst);
        CHECK(worst < 1.005); // never gains more than 0.5%
        CHECK(ratio > 0.97);  // and loses at most 3%
    };
    run(GeometryMode::Axisymmetric, 10.0, 0.5e-3, 40e-6);
    run(GeometryMode::Axisymmetric, 500.0, 0.5e-3, 30e-6);
    run(GeometryMode::PlaneStrain, 500.0, 0.5e-3, 30e-6);
}

// -----------------------------------------------------------------------------
// VALIDATION 3 - SPALL (Stage C). A 2 mm copper flyer hits a 6 mm copper
// target at +-v. Release waves from the flyer's back face and the target's back
// face meet inside the target, one flyer thickness from its back face (x = 4 mm),
// and pull it apart there if the tension (~ the shock pressure) exceeds the
// spall strength (1.3 GPa). Shock pressure: ~0.7 GPa at +-20 m/s (no spall),
// ~3.7 GPa at +-100 m/s (spall).
// -----------------------------------------------------------------------------
TEST_CASE("spall: copper plate cracks one flyer thickness from the back face, only above threshold") {
    auto run = [](double v) {
        Scenario t;
        t.mode = GeometryMode::Axisymmetric;
        t.dx = 0.1e-3;
        t.endTime = 4e-6;
        t.parts.push_back({"flyer", RectShape{{-0.001, 0.0075}, {0.001, 0.0075}}, kCopperOFHC, {v, 0.0}});
        t.parts.push_back({"target", RectShape{{0.003, 0.0075}, {0.003, 0.0075}}, kCopperOFHC, {-v, 0.0}});
        Solver solver(t, defaultMaterials());
        while (!solver.finished() && solver.diagnostics().maxSpeed < 10 * v) solver.step();
        REQUIRE(solver.finished());
        const Particles& P = solver.particles();
        int n = 0;
        double sumX = 0, minX = 1, maxX = -1;
        for (std::size_t i = 0; i < solver.particleCount(); ++i) {
            // Central zone only: release waves from the plate edges arrive later.
            if (P.part[i] != 1 || !P.failed[i] || P.pos[i].y > 0.004) continue;
            ++n;
            sumX += P.pos[i].x;
            minX = std::min(minX, P.pos[i].x);
            maxX = std::max(maxX, P.pos[i].x);
        }
        return std::tuple<int, double, double, double>{n, n ? sumX / n : 0.0, minX, maxX};
    };
    const auto [nLow, xLow, minLow, maxLow] = run(20.0);
    MESSAGE("+-20 m/s: ", nLow, " failed particles in the target centre");
    CHECK(nLow == 0);
    const auto [nHigh, xHigh, minHigh, maxHigh] = run(100.0);
    MESSAGE("+-100 m/s: ", nHigh, " failed, mean x ", xHigh * 1e3, " mm (theory 4.0), from ", minHigh * 1e3, " to ",
            maxHigh * 1e3, " mm");
    CHECK(nHigh > 50);
    CHECK(xHigh == Approx(4.0e-3).epsilon(0.1).scale(0));
}

