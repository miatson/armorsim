// Stage E tests: validation against experiments.
#include <doctest/doctest.h>

#include <algorithm>
#include <filesystem>

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
