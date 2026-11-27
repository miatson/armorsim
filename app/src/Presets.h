#pragma once
// -----------------------------------------------------------------------------
// Presets.h - ready-made scenarios. Stage D replaces this with a full
// scenario builder (pick projectile, armor, angle, layers...).
// -----------------------------------------------------------------------------
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include <armor/Library.h>
#include <armor/Material.h>
#include <armor/Probes.h>
#include <armor/Scenario.h>

struct PresetParams {
    float velocity = 5.0f;   // m/s
    float dxMm = 0.5f;       // particle spacing, mm
    float endTimeUs = 50.0f; // µs
};

struct Preset {
    std::string name;
    std::string description;
    PresetParams defaults;
    float maxVelocity;
    std::function<armor::Scenario(const PresetParams&)> build;
    // Expected results from textbook theory, shown next to the simulation.
    std::function<std::string(const PresetParams&, const std::vector<armor::Material>&)> theory;
    // Gauges recorded during the run (scenario files may define them).
    std::vector<armor::ProbeSpec> probes = {};
    double probeInterval = 0.0; // s; 0 = end time / 1000
};

const std::vector<Preset>& presets();

// Where scenario files and the projectile/armour library are read from:
// $MIATAPHYSIM_SCENARIOS / $MIATAPHYSIM_LIBRARY, else ./scenarios and
// ./library, else the folders in the source tree.
std::filesystem::path scenarioDirectory();
std::filesystem::path libraryDirectory();
// The projectile and armour library (loaded once).
const armor::Library& appLibrary();
