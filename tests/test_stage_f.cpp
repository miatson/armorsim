// Stage F tests: ceramics (Johnson-Holmquist JH-2).
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <variant>
#include <vector>

#include <armor/BallisticLimit.h>
#include <armor/Fragments.h>
#include <armor/Solver.h>

using namespace armor;
using doctest::Approx;

TEST_CASE("JH-2 constants are self-consistent: K1..K3, G and HEL reproduce the published PHEL") {
    // In uniaxial strain at the Hugoniot elastic limit the total stress is
    // HEL = P(mu) + (4/3) G mu/(1+mu); solve for mu and compare P(mu) with PHEL.
    for (int id : {kAluminaAD995, kBoronCarbide, kSiliconCarbide}) {
        const Material m = defaultMaterials()[static_cast<std::size_t>(id)];
        CAPTURE(m.name);
        const auto& jh = std::get<JohnsonHolmquist2>(m.strength);
        double lo = 0.0, hi = 0.2;
        for (int k = 0; k < 100; ++k) {
            const double mu = 0.5 * (lo + hi);
            const double total = m.pressure(m.density0 * (1.0 + mu), 0.0) + 4.0 / 3.0 * m.shearModulus * mu / (1.0 + mu);
            (total < jh.HEL ? lo : hi) = mu;
        }
        const double phel = m.pressure(m.density0 * (1.0 + lo), 0.0);
        MESSAGE(m.name, ": PHEL from the EOS ", phel / 1e9, " GPa, published ", jh.PHEL / 1e9, " GPa");
        CHECK(phel == Approx(jh.PHEL).epsilon(0.01));
        CHECK(jh.sigmaHEL == Approx(1.5 * (jh.HEL - jh.PHEL)));
    }
    // Alumina: the intact strength curve passes close to the HEL point.
    const auto& al = std::get<JohnsonHolmquist2>(defaultMaterials()[kAluminaAD995].strength);
    CHECK(al.A * std::pow(1.0 + al.T / al.PHEL, al.N) == Approx(1.0).epsilon(0.1));
}

TEST_CASE("ceramic tile: crushed core under the impact, intact front away from it, energy conserved") {
    // Steel ball (10 mm, 500 m/s) into a 10 mm alumina tile. Mapped damage
    // (mean per 2 mm x 3 mm cell) shows a STRUCTURED pattern, not noise: a
    // fully crushed cone under the ball through the whole thickness, cracking
    // that spreads outward through the middle and back of the tile (radial
    // cracks from back-face tension), and an intact front surface away from
    // the contact. That is how real unbacked ceramic tiles fail.
    Scenario s;
    s.mode = GeometryMode::Axisymmetric;
    s.dx = 0.25e-3;
    s.endTime = 25e-6;
    s.parts.push_back({"ball", DiscShape{{-5.25e-3, 0.0}, 5e-3}, kSteelRHA, {500.0, 0.0}});
    s.parts.push_back({"alumina", RectShape{{5e-3, 15e-3}, {5e-3, 15e-3}}, kAluminaAD995, {}});
    Solver solver(s, defaultMaterials());
    const Diagnostics d0 = solver.diagnostics();
    double worst = 1.0;
    while (!solver.finished() && solver.diagnostics().maxSpeed < 2500.0) {
        solver.step();
        const Diagnostics d = solver.diagnostics();
        worst = std::max(worst, (d.kinetic + d.internal) / (d0.kinetic + d0.internal));
    }
    REQUIRE(solver.finished());
    const Particles& P = solver.particles();
    double coreSum = 0, frontSum = 0, maxBulking = 0;
    int coreN = 0, frontN = 0;
    for (std::size_t i = 0; i < solver.particleCount(); ++i) {
        if (P.part[i] != 1) continue;
        maxBulking = std::max(maxBulking, P.bulking[i]);
        if (P.pos[i].y > 3e-3 && P.pos[i].y < 6e-3 && P.pos[i].x > 2e-3 && P.pos[i].x < 6e-3) { coreSum += P.damage[i]; ++coreN; }
        if (P.pos[i].x < 2e-3 && P.pos[i].y > 15e-3) { frontSum += P.damage[i]; ++frontN; }
    }
    const Diagnostics d1 = solver.diagnostics();
    const double ratio = (d1.kinetic + d1.internal) / (d0.kinetic + d0.internal);
    MESSAGE("core damage ", coreSum / coreN, ", front surface away from impact ", frontSum / frontN, ", max bulking ",
            maxBulking / 1e9, " GPa, energy highest ", worst, ", final ", ratio);
    REQUIRE(coreN > 0);
    REQUIRE(frontN > 0);
    CHECK(coreSum / coreN > 0.9);   // crushed cone under the ball
    CHECK(frontSum / frontN < 0.2); // front surface far from the contact stays intact
    CHECK(maxBulking > 0.0);        // cracking raised the pressure (bulking)
    CHECK(worst < 1.005);
    CHECK(ratio > 0.95);
}

// Equal-weight comparison (64 kg/m2), run on demand:
//   armor_tests -tc="DIAG equal-weight*" --no-skip -s
// Result (0.2 mm): A 686 m/s, B 682 m/s, C 641 m/s - all perforated; the ceramic
// advantage seen in real armour does NOT show yet (open, see README Stage F).
TEST_CASE("DIAG equal-weight armour comparison" * doctest::skip()) {
    auto shot = [](const char* label, std::vector<Part> armour) {
        Scenario s;
        s.mode = GeometryMode::Axisymmetric;
        s.dx = 0.2e-3;
        s.endTime = 120e-6;
        s.parts.push_back({"tungsten rod", RectShape{{-15.2e-3, 1.5e-3}, {15e-3, 1.5e-3}}, kTungstenHeavyAlloy, {900.0, 0.0}});
        for (Part& p : armour) s.parts.push_back(p);
        const ShotResult r = fireShot(s, defaultMaterials());
        MESSAGE(std::string(label), ": ", std::string(r.blewUp ? "BLEW UP" : (r.perforated ? "perforated" : "stopped")),
                ", rod leaves at ", r.residualVelocity, " m/s (plate ", r.plateVelocity, " m/s) after ", r.timeSimulated * 1e6,
                " us, ", r.wallSeconds, " s");
    };
    shot("A alumina 10 + aluminium 10", {{"alumina", RectShape{{5e-3, 15e-3}, {5e-3, 15e-3}}, kAluminaAD995, {}},
                                          {"aluminium", RectShape{{15e-3, 15e-3}, {5e-3, 15e-3}}, kAluminium6061, {}}});
    shot("B aluminium 23.7", {{"aluminium", RectShape{{11.85e-3, 15e-3}, {11.85e-3, 15e-3}}, kAluminium6061, {}}});
    shot("C steel RHA 8.2", {{"steel", RectShape{{4.1e-3, 15e-3}, {4.1e-3, 15e-3}}, kSteelRHA, {}}});
}

// Residual MASS and speed of the rod after the equal-weight targets, run long.
//   armor_tests -tc="DIAG residual mass*" --no-skip -s
TEST_CASE("DIAG residual mass equal-weight" * doctest::skip()) {
    auto shot = [](const char* label, std::vector<Part> armour, double backFace, std::vector<Material> mats = defaultMaterials()) {
        Scenario s;
        s.mode = GeometryMode::Axisymmetric;
        s.dx = 0.2e-3;
        s.endTime = 250e-6;
        s.parts.push_back({"tungsten rod", RectShape{{-15.2e-3, 1.5e-3}, {15e-3, 1.5e-3}}, kTungstenHeavyAlloy, {900.0, 0.0}});
        for (Part& p : armour) s.parts.push_back(p);
        Solver solver(s, mats);
        double m0 = 0;
        for (std::size_t i = 0; i < solver.particleCount(); ++i)
            if (solver.particles().part[i] == 0) m0 += solver.particles().mass[i];
        while (!solver.finished() && solver.diagnostics().maxSpeed < 4000) solver.step();
        const FragmentStats fs = solver.fragments();
        double through = 0, throughMom = 0, biggest = 0, biggestV = 0;
        for (const Fragment& f : fs.pieces) {
            if (f.body != 0 || f.particles < 3) continue;
            if (f.mass > biggest) { biggest = f.mass; biggestV = f.velocity.x; }
            if (f.centroid.x > backFace) { through += f.mass; throughMom += f.mass * f.velocity.x; }
        }
        char b[300];
        std::snprintf(b, sizeof b, "%s: rod mass through the armour %.1f%% (moving %.0f m/s), largest rod piece %.1f%% at %.0f m/s",
                      label, 100 * through / m0, through > 0 ? throughMom / through : 0.0, 100 * biggest / m0, biggestV);
        MESSAGE(std::string(b));
    };
    shot("A alumina 10 + aluminium 10", {{"alumina", RectShape{{5e-3, 15e-3}, {5e-3, 15e-3}}, kAluminaAD995, {}},
                                          {"aluminium", RectShape{{15e-3, 15e-3}, {5e-3, 15e-3}}, kAluminium6061, {}}}, 20e-3);
    shot("B aluminium 23.7", {{"aluminium", RectShape{{11.85e-3, 15e-3}, {11.85e-3, 15e-3}}, kAluminium6061, {}}}, 23.7e-3);
    shot("C steel RHA 8.2", {{"steel", RectShape{{4.1e-3, 15e-3}, {4.1e-3, 15e-3}}, kSteelRHA, {}}}, 8.2e-3);
}

TEST_CASE("DIAG ceramic variants" * doctest::skip()) {
    auto run = [](const char* label, std::vector<Material> mats, bool bonded = false) {
        Scenario s;
        s.mode = GeometryMode::Axisymmetric;
        s.dx = 0.2e-3;
        s.endTime = 250e-6;
        s.parts.push_back({"tungsten rod", RectShape{{-15.2e-3, 1.5e-3}, {15e-3, 1.5e-3}}, kTungstenHeavyAlloy, {900.0, 0.0}});
        s.parts.push_back({"alumina", RectShape{{5e-3, 15e-3}, {5e-3, 15e-3}}, kAluminaAD995, {}});
        s.parts.push_back({"aluminium", RectShape{{15e-3, 15e-3}, {5e-3, 15e-3}}, kAluminium6061, {}});
        if (bonded) s.parts[1].contactGroup = s.parts[2].contactGroup = 0; // glued: one body
        Solver solver(s, mats);
        double m0 = 0;
        const std::uint16_t rodBody = bodyId(s, 0);
        for (std::size_t i = 0; i < solver.particleCount(); ++i)
            if (solver.particles().part[i] == rodBody) m0 += solver.particles().mass[i];
        while (!solver.finished() && solver.diagnostics().maxSpeed < 4000) solver.step();
        double through = 0, mom = 0;
        for (const Fragment& f : solver.fragments().pieces)
            if (f.body == rodBody && f.particles >= 3 && f.centroid.x > 20e-3) { through += f.mass; mom += f.mass * f.velocity.x; }
        char b[200];
        std::snprintf(b, sizeof b, "%s: rod mass through %.1f%% at %.0f m/s, relative energy %.3f", label, 100 * through / m0,
                      through > 0 ? mom / through : 0.0, through > 0 ? (through / m0) * (mom / through) * (mom / through) / 505000.0 : 0.0);
        MESSAGE(std::string(b));
    };
    // Results so far: as built 1.00; rubble NOT capped 0.956; never cracks 0.215.
    run("A, tile BONDED to backing", defaultMaterials(), true);
}
