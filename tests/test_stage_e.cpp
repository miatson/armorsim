// Stage E tests: validation against experiments.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include <armor/BallisticLimit.h>
#include <armor/ScenarioFile.h>
#include <armor/Solver.h>
#include <armor/Validation.h>

using namespace armor;
using doctest::Approx;

TEST_CASE("Lanz-Odermatt fit: known values and trends") {
    const double rw = 17600.0, rs = 7850.0;
    // The well-known ballpark: an L/D 10 tungsten rod at 1.5 km/s into RHA
    // gets ~0.85 rod lengths deep.
    CHECK(lanzOdermattTungstenRHA(10.0, rw, rs, kSteelRHABrinell, 1500.0) == Approx(0.852).epsilon(0.005));
    // Faster = deeper, approaching the hydrodynamic-like plateau.
    double prev = 0.0;
    for (double v = 800.0; v <= 3000.0; v += 200.0) {
        const double pl = lanzOdermattTungstenRHA(10.0, rw, rs, kSteelRHABrinell, v);
        CHECK(pl > prev);
        prev = pl;
    }
    // Longer rods (larger L/D) get fewer rod lengths deep (the L/D effect).
    CHECK(lanzOdermattTungstenRHA(30.0, rw, rs, kSteelRHABrinell, 1500.0) <
          lanzOdermattTungstenRHA(10.0, rw, rs, kSteelRHABrinell, 1500.0));
    // Harder armour stops it sooner.
    CHECK(lanzOdermattTungstenRHA(10.0, rw, rs, 400.0, 1500.0) < lanzOdermattTungstenRHA(10.0, rw, rs, 250.0, 1500.0));
}

// Slow (~2 min): run with  armor_tests -tc="VALIDATION*" --no-skip
TEST_CASE("VALIDATION long rod: tungsten L/D 10 into thick RHA at 1500 m/s vs Lanz-Odermatt" * doctest::skip()) {
    const ScenarioFile f = loadScenarioFile(std::filesystem::path(ARMOR_SCENARIO_DIR) / "validation_long_rod_ld10.json",
                                            defaultMaterials());
    Solver solver(f.scenario, defaultMaterials());
    while (!solver.finished() && solver.diagnostics().maxSpeed < 5000.0) solver.step();
    REQUIRE(solver.finished());
    const Particles& P = solver.particles();
    double deepest = -1.0;
    for (std::size_t i = 0; i < solver.particleCount(); ++i)
        if (P.part[i] == 0) deepest = std::max(deepest, P.pos[i].x);
    const double depth = deepest + 0.5 * f.scenario.dx; // target front face at x = 0
    const double pl = depth / 0.1;
    const double expected = lanzOdermattTungstenRHA(10.0, 17600.0, 7850.0, kSteelRHABrinell, 1500.0);
    MESSAGE("P/L simulated ", pl, ", Lanz-Odermatt ", expected);
    CHECK(pl == Approx(expected).epsilon(0.10).scale(0));
}

TEST_CASE("Recht-Ipson fit recovers a known ballistic limit") {
    // Synthetic shots on v_r = 0.8 sqrt(v^2 - 200^2): the fit must return 200 m/s and 0.8.
    std::vector<ShotResult> shots;
    for (double v : {150.0, 190.0, 220.0, 260.0, 320.0}) {
        ShotResult s;
        s.speed = v;
        s.perforated = v > 200.0;
        s.residualVelocity = s.perforated ? 0.8 * std::sqrt(v * v - 200.0 * 200.0) : 0.0;
        shots.push_back(s);
    }
    double limit = 0, a = 0;
    REQUIRE(rechtIpsonFit(shots, limit, a));
    CHECK(limit == Approx(200.0).epsilon(1e-6));
    CHECK(a == Approx(0.8).epsilon(1e-6));
}

TEST_CASE("fireShot: a slow ball stops on a plate, a fast one goes through a thin sheet") {
    const auto mats = defaultMaterials();
    auto shot = [&](double v, double thicknessMm) {
        Scenario s;
        s.mode = GeometryMode::Axisymmetric;
        s.dx = 0.25e-3;
        s.endTime = 60e-6;
        s.parts.push_back({"ball", DiscShape{{-2.75e-3, 0.0}, 2.5e-3}, kSteelRHA, {v, 0.0}});
        s.parts.push_back({"sheet", RectShape{{0.5 * thicknessMm * 1e-3, 7.5e-3}, {0.5 * thicknessMm * 1e-3, 7.5e-3}}, kAluminium6061, {}});
        return fireShot(s, mats);
    };
    const ShotResult slow = shot(100.0, 5.0);
    MESSAGE("slow: perforated ", slow.perforated, ", projectile ", slow.residualVelocity, " m/s, plate ", slow.plateVelocity, " m/s");
    CHECK(!slow.perforated);
    CHECK(!slow.blewUp);
    const ShotResult fast = shot(1500.0, 1.0);
    MESSAGE("fast: perforated ", fast.perforated, ", residual ", fast.residualVelocity, " m/s after ", fast.timeSimulated * 1e6, " us");
    CHECK(fast.perforated);
    CHECK(fast.residualVelocity > 500.0);
    CHECK(fast.residualVelocity < 1500.0);
}

// Slow (several minutes): run with  armor_tests -tc="VALIDATION Borvik*" --no-skip -s
TEST_CASE("VALIDATION Borvik: ballistic limit of 12 mm Weldox 460 E vs blunt projectile (exp. 184.5 m/s)" * doctest::skip()) {
    const auto mats = defaultMaterials();
    const ScenarioFile f = loadScenarioFile(std::filesystem::path(ARMOR_SCENARIO_DIR) / "validation_borvik_blunt.json", mats);
    const double v0 = f.scenario.parts[0].velocity.x;
    auto make = [&](double v) {
        Scenario s = f.scenario;
        s.parts[0].velocity = {v, 0.0};
        (void)v0;
        return s;
    };
    const BallisticLimitResult r = findBallisticLimit(make, 150.0, 250.0, 5, mats, [](const ShotResult& s) {
        MESSAGE(s.speed, " m/s: ", std::string(s.blewUp ? "BLEW UP" : (s.perforated ? "perforated" : "stopped")), ", residual ",
                s.residualVelocity, " m/s, plate ", s.plateVelocity, " m/s, ", s.timeSimulated * 1e6, " us, ", s.wallSeconds, " s");
    });
    REQUIRE(r.bracketed());
    MESSAGE("V50 = ", r.v50(), " m/s (", r.highestStopped, " .. ", r.lowestPerforated, "); experiment 184.5 m/s");
    if (r.hasFit) MESSAGE("Recht-Ipson fit: v_bl = ", r.fitLimit, " m/s, a = ", r.fitA);
    // Measured (0.4 mm spacing): V50 207.8 m/s (+12.6%), Recht-Ipson 203.1 m/s (+10%) - see README.
    CHECK(r.v50() == Approx(184.5).epsilon(0.15).scale(0));
}
