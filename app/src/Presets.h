#pragma once
// -----------------------------------------------------------------------------
// Presets.h - ready-made scenarios. Stage D replaces this with a full
// scenario builder (pick projectile, armor, angle, layers...).
// -----------------------------------------------------------------------------
#include <functional>
#include <string>
#include <vector>

#include <armor/Material.h>
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
};

const std::vector<Preset>& presets();
