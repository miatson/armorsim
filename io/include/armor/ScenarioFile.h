#pragma once
// -----------------------------------------------------------------------------
// ScenarioFile.h - scenarios described in JSON files (Stage D).
//
// Friendly units in the file (mm, µs, m/s, K, degrees); SI inside the solver.
// Materials are named by their key ("steel_rha", "copper_ofhc", ...) or full
// name. Unknown keys are an ERROR, not ignored: a silently ignored typo in a
// physics input is worse than a message. See scenarios/README.md for the format.
//
//   {
//     "name": "Copper spall plate",
//     "description": "optional text shown in the app",
//     "mode": "axisymmetric" | "plane_strain",
//     "spacing_mm": 0.1, "end_time_us": 6, "frames": 200,
//     "settings": { "failure": true, ... },
//     "parts": [ { "name": "Flyer", "material": "copper_ofhc",
//                  "shape": { "type": "cylinder", "x_mm": [-2, 0], "radius_mm": 7.5 },
//                  "velocity_mps": [100, 0], "temperature_K": 293.15,
//                  "contact_group": -1 } ],
//     "probes": [ { "name": "back face", "at_mm": [5.95, 0.5],
//                   "quantities": ["velocity_x", "pressure"] } ],
//     "probe_interval_us": 0.01
//   }
//
// Shapes: "rect" (center_mm, size_mm, angle_deg), "disc" (center_mm,
// radius_mm), "cylinder" (axisymmetric: x_mm [from, to], radius_mm - a solid
// round rod or plate on the axis), "sphere" (axisymmetric: center_x_mm,
// radius_mm - a ball on the axis).
// -----------------------------------------------------------------------------
#include <filesystem>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <armor/Material.h>
#include <armor/Scenario.h>

#include "armor/Probes.h"

namespace armor {

struct ScenarioFile {
    Scenario scenario;
    std::string description;
    std::vector<ProbeSpec> probes;
    double probeInterval = 0.0; // s; 0 = endTime / 1000
};

// Thrown with a message that names the file and the offending key.
struct ScenarioFileError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

ScenarioFile parseScenarioJson(const std::string& text, std::span<const Material> materials,
                               const std::string& sourceName = "scenario");
ScenarioFile loadScenarioFile(const std::filesystem::path& path, std::span<const Material> materials);

} // namespace armor
