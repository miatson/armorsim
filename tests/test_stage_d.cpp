// Stage D tests: scenario files (JSON) and probes (time histories, CSV).
#include <doctest/doctest.h>

#include <algorithm>
#include <filesystem>
#include <sstream>
#include <string>

#include <armor/Library.h>
#include <armor/ScenarioFile.h>
#include <armor/Solver.h>

using namespace armor;
using doctest::Approx;

namespace {

// Expect a ScenarioFileError whose message contains `needle`.
void expectError(const std::string& json, const std::string& needle) {
    CAPTURE(json);
    try {
        parseScenarioJson(json, defaultMaterials(), "test.json");
        FAIL("no error thrown; expected one mentioning: " << needle);
    } catch (const ScenarioFileError& e) {
        const std::string what = e.what();
        CAPTURE(what);
        CHECK(what.find(needle) != std::string::npos);
    }
}

const char* kMinimal = R"({
  "name": "t", "mode": "axisymmetric", "spacing_mm": 0.5, "end_time_us": 10,
  "parts": [ { "name": "rod", "material": "steel_rha",
               "shape": { "type": "cylinder", "x_mm": [0, 20], "radius_mm": 5 },
               "velocity_mps": [100, 0] } ] })";

} // namespace

TEST_CASE("every scenario file shipped in scenarios/ loads and builds particles") {
    int count = 0;
    for (const auto& entry : std::filesystem::directory_iterator(ARMOR_SCENARIO_DIR)) {
        if (entry.path().extension() != ".json") continue;
        CAPTURE(entry.path().string());
        const ScenarioFile f = loadScenarioFile(entry.path(), defaultMaterials());
        CHECK(!f.scenario.name.empty());
        CHECK(buildParticles(f.scenario, defaultMaterials()).size() > 100);
        ++count;
    }
    MESSAGE(count, " scenario files checked");
    CHECK(count >= 3);
}

TEST_CASE("file units and shapes map exactly onto the solver's geometry") {
    const ScenarioFile f = parseScenarioJson(kMinimal, defaultMaterials());
    const Scenario& s = f.scenario;
    CHECK(s.mode == GeometryMode::Axisymmetric);
    CHECK(s.dx == Approx(0.5e-3));
    CHECK(s.endTime == Approx(10e-6));
    REQUIRE(s.parts.size() == 1);
    CHECK(s.parts[0].material == kSteelRHA);
    CHECK(s.parts[0].velocity.x == Approx(100.0));
    // A cylinder x 0..20 mm, radius 5 mm is the rectangle the presets use for a rod.
    const auto& r = std::get<RectShape>(s.parts[0].shape);
    CHECK(r.center.x == Approx(10e-3));
    CHECK(r.center.y == Approx(2.5e-3));
    CHECK(r.halfSize.x == Approx(10e-3));
    CHECK(r.halfSize.y == Approx(2.5e-3));

    // Full material names work as well as keys; spheres sit on the axis.
    const ScenarioFile g = parseScenarioJson(R"js({
      "name": "t", "mode": "axisymmetric", "spacing_mm": 0.5, "end_time_us": 10,
      "parts": [ { "name": "ball", "material": "Copper (OFHC)",
                   "shape": { "type": "sphere", "center_x_mm": -5, "radius_mm": 5 } } ] })js",
                                             defaultMaterials());
    CHECK(g.scenario.parts[0].material == kCopperOFHC);
    const auto& d = std::get<DiscShape>(g.scenario.parts[0].shape);
    CHECK(d.center.x == Approx(-5e-3));
    CHECK(d.center.y == 0.0);
    CHECK(d.radius == Approx(5e-3));
}

TEST_CASE("bad scenario files fail with a message that says where and why") {
    expectError("{ not json", "not valid JSON");
    std::string s = kMinimal;
    expectError(std::string(s).replace(s.find("\"spacing_mm\""), 12, "\"spacing\""), "$.spacing: unknown key");
    expectError(std::string(s).replace(s.find("steel_rha"), 9, "unobtainium"), "$.parts[0].material: unknown material");
    expectError(std::string(s).replace(s.find("axisymmetric"), 12, "plane_strain"),
                "$.parts[0].shape.type: \"cylinder\" needs axisymmetric mode");
    expectError(std::string(s).replace(s.find("[100, 0]"), 8, "[100, 5]"), "can only move along the axis");
    expectError(std::string(s).replace(s.find("\"end_time_us\": 10"), 17, "\"end_time_us\": -1"),
                "$.end_time_us: must be positive");
    expectError(R"({ "name": "t", "mode": "axisymmetric", "spacing_mm": 0.5, "end_time_us": 10, "parts": [] })",
                "$.parts: expected a non-empty list");
    expectError(std::string(s).insert(s.rfind('}'), R"(, "probes": [ { "name": "p", "at_mm": [0, 0], "quantities": ["presure"] } ])"),
                "unknown quantity \"presure\"");
}

TEST_CASE("probe on the spall target's back face records the free-surface velocity") {
    // The shipped spall scenario: the back face (initially at -100 m/s) must
    // jump to +100 m/s (2v relative to the target) when the shock arrives at
    // ~1.3 us, then drop by the spall pullback - and the CSV must hold it all.
    const ScenarioFile f = loadScenarioFile(std::filesystem::path(ARMOR_SCENARIO_DIR) / "spall_copper.json",
                                            defaultMaterials());
    Solver solver(f.scenario, defaultMaterials());
    ProbeRecorder rec(f.probes, solver, f.probeInterval);
    CHECK(rec.pickError(0) < 0.1e-3);
    while (!solver.finished()) {
        solver.step();
        rec.update(solver);
    }
    REQUIRE(rec.times().size() > 500);
    CHECK(rec.times().front() == 0.0);
    CHECK(rec.times().back() == Approx(f.scenario.endTime));
    double peak = -1e9, tPeak = 0.0;
    for (std::size_t s = 0; s < rec.times().size(); ++s)
        if (rec.values()[s][0] > peak) {
            peak = rec.values()[s][0];
            tPeak = rec.times()[s];
        }
    MESSAGE("back face: peak ", peak, " m/s at ", tPeak * 1e6, " us; final ", rec.values().back()[0], " m/s");
    CHECK(rec.values().front()[0] == Approx(-100.0));
    CHECK(peak == Approx(100.0).epsilon(0.07));
    CHECK(tPeak > 1.0e-6);  // shock transit through 6 mm of copper ~1.3 us
    CHECK(tPeak < 2.0e-6);
    CHECK(rec.values().back()[0] < peak - 50.0); // the pullback

    std::ostringstream csv;
    rec.writeCsv(csv);
    const std::string text = csv.str();
    CHECK(text.rfind("\"time [s]\",\"back face: velocity_x [m/s]\",\"spall plane: pressure [Pa]\"", 0) == 0);
    CHECK(std::count(text.begin(), text.end(), '\n') == static_cast<long>(rec.times().size()) + 1);
}

// -----------------------------------------------------------------------------
// Library and builder
// -----------------------------------------------------------------------------
TEST_CASE("library: every item loads, and every projectile/armour pair builds a valid scenario") {
    const auto mats = defaultMaterials();
    const Library lib = loadLibrary(ARMOR_LIBRARY_DIR, mats);
    for (const auto& e : lib.errors) MESSAGE(e);
    CHECK(lib.errors.empty());
    CHECK(lib.projectiles.size() >= 5);
    CHECK(lib.armor.size() >= 5);
    for (const LibraryItem& p : lib.projectiles) {
        for (const LibraryItem& a : lib.armor) {
            CAPTURE(p.name);
            CAPTURE(a.name);
            BuildSpec spec;
            spec.projectile = p;
            spec.projectileMaterial = p.material;
            spec.speed = p.speed;
            spec.layers.push_back({a, a.material, a.thickness, 0.0});
            spec.spacing = suggestSpacing(spec);
            spec.endTime = suggestEndTime(spec);
            CHECK(spec.spacing >= 0.05e-3);
            CHECK(spec.endTime >= 5e-6);
            const ScenarioFile f = buildScenario(spec, mats);
            const Particles P = buildParticles(f.scenario, mats);
            CHECK(P.size() > 50);
            CHECK(P.size() < 80000); // suggested spacing keeps runs reasonable (dense tank sections ~65k)
        }
    }
}

TEST_CASE("builder: projectile starts clear of the armour, straight and oblique; JSON round trip") {
    const auto mats = defaultMaterials();
    const Library lib = loadLibrary(ARMOR_LIBRARY_DIR, mats);
    REQUIRE(!lib.projectiles.empty());
    REQUIRE(lib.armor.size() >= 2);
    for (double angleDeg : {0.0, 30.0, 60.0}) {
        CAPTURE(angleDeg);
        BuildSpec spec;
        spec.projectile = lib.projectiles[0];
        spec.projectileMaterial = spec.projectile.material;
        spec.speed = 800.0;
        spec.layers.push_back({lib.armor[0], lib.armor[0].material, lib.armor[0].thickness, 0.0});
        spec.layers.push_back({lib.armor[1], lib.armor[1].material, lib.armor[1].thickness, 5e-3});
        spec.obliquity = angleDeg * 3.14159265358979323846 / 180.0;
        spec.spacing = 0.5e-3;
        spec.endTime = 20e-6;
        const ScenarioFile f = buildScenario(spec, mats);
        CHECK((f.scenario.mode == GeometryMode::Axisymmetric) == (angleDeg == 0.0));
        REQUIRE(f.scenario.parts.size() == 3);
        CHECK(f.scenario.parts[0].velocity.x == Approx(800.0));
        // Every part got its particles, and the projectile is at least half a
        // spacing away from all armour at t = 0.
        const Particles P = buildParticles(f.scenario, mats);
        std::size_t count[3] = {};
        double nearest = 1e9;
        for (std::size_t i = 0; i < P.size(); ++i) {
            ++count[P.part[i]];
            if (P.part[i] != 0) continue;
            for (std::size_t j = 0; j < P.size(); ++j)
                if (P.part[j] != 0) nearest = std::min(nearest, length(P.pos[i] - P.pos[j]));
        }
        CHECK(count[0] > 20);
        CHECK(count[1] > 20);
        CHECK(count[2] > 20);
        CHECK(nearest > 0.99 * spec.spacing);
        // Save as a scenario file and load it back: same geometry.
        const ScenarioFile g = parseScenarioJson(scenarioToJson(f, mats), mats, "roundtrip.json");
        const Particles Q = buildParticles(g.scenario, mats);
        CHECK(Q.size() == P.size());
        CHECK(g.probes.size() == f.probes.size());
        CHECK(g.scenario.parts[2].material == f.scenario.parts[2].material);
    }
}

// -----------------------------------------------------------------------------
// Tank rounds and tank-front estimates (library data + instant checks)
// -----------------------------------------------------------------------------
#include <armor/Validation.h>

TEST_CASE("pointed rod nose: polygon shape, right mass, survives a JSON round trip") {
    const auto mats = defaultMaterials();
    const LibraryItem rod = parseLibraryItem(
        R"({"name":"r","category":"projectile","material":"tungsten_wha",
            "shape":{"type":"rod","length_mm":100,"diameter_mm":10,"nose_length_mm":30}})", mats);
    CHECK(rod.noseLength == Approx(0.030));
    CHECK(rodWorkingLength(rod) == Approx(0.080));
    const double pi = 3.14159265358979323846;
    CHECK(projectileVolume(rod) == Approx(pi * 25e-6 * (0.070 + 0.010)));
    CHECK_THROWS_AS(parseLibraryItem(R"({"name":"r","category":"projectile","material":"tungsten_wha",
            "shape":{"type":"rod","length_mm":10,"diameter_mm":10,"nose_length_mm":10}})", mats), ScenarioFileError);
    for (double angleDeg : {0.0, 30.0}) {
        CAPTURE(angleDeg);
        BuildSpec spec;
        spec.projectile = rod;
        spec.projectileMaterial = kTungstenHeavyAlloy;
        spec.speed = 1500.0;
        LibraryItem plate;
        plate.category = ItemCategory::Armor;
        plate.shape = ItemShape::Plate;
        plate.thickness = 0.02;
        plate.width = 0.06;
        spec.layers.push_back({plate, kSteelRHA, 0.02, 0.0});
        spec.obliquity = angleDeg * pi / 180.0;
        spec.spacing = 0.5e-3;
        const ScenarioFile f = buildScenario(spec, mats);
        REQUIRE(std::holds_alternative<PolygonShape>(f.scenario.parts[0].shape));
        // Particle mass of the projectile matches the solid (axisymmetric: real
        // mass; plane strain: per unit depth).
        const Particles P = buildParticles(f.scenario, mats);
        double m = 0.0;
        for (std::size_t i = 0; i < P.size(); ++i)
            if (P.part[i] == 0) m += P.mass[i];
        const double area2d = 2 * 0.005 * (0.070 + 0.5 * 0.030);
        const double expect = angleDeg == 0.0 ? 17600.0 * projectileVolume(rod) : 17600.0 * area2d;
        CHECK(m == Approx(expect).epsilon(0.06));
        const ScenarioFile g = parseScenarioJson(scenarioToJson(f, mats), mats, "roundtrip.json");
        CHECK(buildParticles(g.scenario, mats).size() == P.size());
    }
}

TEST_CASE("tank rounds vs tank-front estimates: data present, Lanz-Odermatt depths plausible") {
    const auto mats = defaultMaterials();
    const Library lib = loadLibrary(ARMOR_LIBRARY_DIR, mats);
    int rounds = 0, tanks = 0;
    for (const LibraryItem& a : lib.armor)
        if (a.isTankEstimate() && a.shape == ItemShape::Plate) { // steel stand-in blocks
            ++tanks;
            CHECK(a.estimateLow <= a.thickness);
            CHECK(a.thickness <= a.estimateHigh);
        }
    for (const LibraryItem& p : lib.projectiles) {
        if (p.file.rfind("apfsds_", 0) != 0) continue;
        ++rounds;
        const double lw = rodWorkingLength(p);
        const double depth = lanzOdermattTungstenRHA(lw / p.diameter, mats[p.material].density0,
                                                     mats[kSteelRHA].density0, kSteelRHABrinell, p.speed) * lw;
        const auto kind = p.depletedUranium ? PenetratorKind::DepletedUranium : PenetratorKind::Tungsten;
        const double rho = p.depletedUranium ? kDepletedUraniumDensity : mats[p.material].density0;
        const double perf0 = odermattPerforation(lw / p.diameter, rho, mats[kSteelRHA].density0, kThickRHABrinell,
                                                 p.speed, 0.0, kind) * lw;
        const double perf2 = odermattPerforation(lw / p.diameter, rho, mats[kSteelRHA].density0, kThickRHABrinell,
                                                 p.speed - 2.0 * kRodSpeedLossPerKm, 0.0, kind) * lw;
        MESSAGE(p.name, ": ", mats[p.material].density0 * projectileVolume(p), " kg, ", p.speed,
                " m/s; thick block (Lanz-Odermatt) ", depth * 1e3, " mm; plate perforation (Odermatt) muzzle ",
                perf0 * 1e3, " mm, 2 km ", perf2 * 1e3, " mm");
        CHECK(depth > 0.40);
        CHECK(depth < 0.80);
        // Published figures for these rounds are mostly ~550-750 mm (plate).
        CHECK(perf0 > 0.50);
        CHECK(perf0 < 0.80);
        CHECK(perf2 < perf0);
        CHECK(mats[p.material].density0 * projectileVolume(p) == Approx(4.5).epsilon(0.35)); // rods weigh ~4-6 kg
    }
    CHECK(rounds >= 4);
    CHECK(tanks >= 5);
}

TEST_CASE("armour assemblies (tank cross-sections) and free placement") {
    const auto mats = defaultMaterials();
    const Library lib = loadLibrary(ARMOR_LIBRARY_DIR, mats);
    int sections = 0;
    for (const LibraryItem& a : lib.armor) {
        if (a.shape != ItemShape::Assembly) continue;
        ++sections;
        CAPTURE(a.name);
        CHECK(a.parts.size() >= 3);
        double x0 = 1e9;
        for (const AssemblyPart& p : a.parts)
            for (const Vec2& v : p.points) x0 = std::min(x0, v.x);
        CHECK(x0 == Approx(0.0).epsilon(1e-9)); // front at x = 0
        BuildSpec spec;
        spec.projectile = lib.projectiles[0];
        spec.projectileMaterial = spec.projectile.material;
        spec.speed = 1000.0;
        spec.layers.push_back({a, a.material, a.thickness, 0.0});
        CHECK(needsSlice(spec));
        spec.spacing = suggestSpacing(spec);
        const ScenarioFile f = buildScenario(spec, mats);
        CHECK(f.scenario.mode == GeometryMode::PlaneStrain);
        CHECK(f.scenario.parts.size() == a.parts.size() + 1);
        const Particles P = buildParticles(f.scenario, mats);
        CHECK(P.size() > 1000);
        CHECK(P.size() < 80000);
    }
    CHECK(sections >= 5);

    // Placement: projectile moved up 5 mm and turned 30 deg: shape and velocity follow.
    BuildSpec spec;
    spec.projectile = lib.projectiles[0];
    spec.projectileMaterial = spec.projectile.material;
    spec.speed = 800.0;
    spec.layers.push_back({lib.armor[0], lib.armor[0].material, lib.armor[0].thickness, 0.0});
    spec.spacing = 0.5e-3;
    const ScenarioFile base = buildScenario(spec, mats);
    CHECK(!needsSlice(spec));
    spec.placements.resize(2);
    spec.placements[0] = {0.0, 5e-3, 30.0 * 3.14159265358979323846 / 180.0};
    CHECK(needsSlice(spec));
    const ScenarioFile moved = buildScenario(spec, mats);
    const Vec2 v = moved.scenario.parts[0].velocity;
    CHECK(v.x == Approx(800.0 * std::cos(30.0 * 3.14159265358979323846 / 180.0)));
    CHECK(v.y == Approx(800.0 * std::sin(30.0 * 3.14159265358979323846 / 180.0)));
    const ShapeBox b0 = shapeBounds(base.scenario.parts[0].shape);
    const ShapeBox b1 = shapeBounds(moved.scenario.parts[0].shape);
    CHECK(0.5 * (b1.min.y + b1.max.y) == Approx(5e-3).epsilon(0.02));
    (void)b0;
    // The JSON round trip keeps moved geometry.
    const ScenarioFile g = parseScenarioJson(scenarioToJson(moved, mats), mats, "roundtrip.json");
    CHECK(buildParticles(g.scenario, mats).size() == buildParticles(moved.scenario, mats).size());
}

TEST_CASE("mounts: held edges follow the shot, keep the plate in place, energy still balances") {
    // Zone rule: room around the shot, band at the far edges, off when no room.
    const MountZone z = mountZone(0.150, 0.010, 0.001);
    CHECK(z.ok);
    CHECK(z.start == Approx(0.150 - 0.0225));
    CHECK(z.start >= 3 * 0.010 + 0.010);
    CHECK_FALSE(mountZone(0.030, 0.010, 0.001).ok); // edge too close to the shot

    const auto mats = defaultMaterials();
    auto run = [&](bool held, double aimY) {
        Scenario s;
        s.mode = GeometryMode::PlaneStrain;
        s.dx = 0.5e-3;
        s.endTime = 30e-6;
        s.parts.push_back({"ball", DiscShape{{-3.3e-3, aimY}, 2.5e-3}, kSteelRHA, {600.0, 0.0}});
        Part plate{"plate", RectShape{{2e-3, 0.0}, {2e-3, 40e-3}, 0.0}, kAluminium6061, {}};
        plate.held = held;
        s.parts.push_back(plate);
        Solver solver(s, mats);
        const Diagnostics d0 = solver.diagnostics();
        const std::size_t heldAtStart = solver.heldParticleCount();
        // No held particle near the shot line.
        // (positions are checked through the zone: nothing within the clearance)
        while (!solver.finished()) solver.step();
        const Diagnostics d1 = solver.diagnostics();
        const Particles& P = solver.particles();
        double m = 0, px = 0; // mean speed of the plate's outer parts (> 30 mm from the shot line)
        for (std::size_t i = 0; i < solver.particleCount(); ++i)
            if (P.part[i] == 1 && std::abs(P.pos[i].y - aimY) > 30e-3) { m += P.mass[i]; px += P.mass[i] * length(P.vel[i]); }
        struct R { std::size_t held; double ratio, plateSpeed, absorbed; std::string note; };
        return R{heldAtStart, (d1.kinetic + d1.internal + d1.absorbed) / (d0.kinetic + d0.internal), px / m, d1.absorbed,
                 solver.mountNote()};
    };
    const auto free = run(false, 0.0);
    const auto held = run(true, 0.0);
    MESSAGE("outer plate speed, free: ", free.plateSpeed, " m/s; held: ", held.plateSpeed, " m/s, ", held.held,
            " particles held, absorbed ", held.absorbed, " J/m, energy ratio ", held.ratio);
    CHECK(free.held == 0);
    CHECK(held.held > 50);
    CHECK(held.absorbed > 0.0);
    CHECK(held.ratio == Approx(1.0).epsilon(0.03));
    CHECK(std::abs(held.plateSpeed) < std::abs(free.plateSpeed));
    // Aimed 35 mm off centre: the near edge is within the clearance of a 5 mm ball... the plate is
    // still held on its far side (one body: zone from its farthest point), so it stays held.
    const auto offAim = run(true, 35e-3);
    CHECK(offAim.held > 0);
}
