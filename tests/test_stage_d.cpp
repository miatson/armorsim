// Stage D tests: scenario files (JSON) and probes (time histories, CSV).
#include <doctest/doctest.h>

#include <algorithm>
#include <filesystem>
#include <sstream>
#include <string>

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
