#pragma once
// -----------------------------------------------------------------------------
// Library.h - projectiles and armour as separate building blocks (Stage D).
//
// Library items live in JSON files under library/projectiles and
// library/armor. An item is a SHAPE plus a default material; the user picks
// the material per item, so one "10 mm plate" can be tested in steel,
// aluminium, titanium... A BuildSpec says what to fire at what: one
// projectile, a speed, and any number of armour layers with gaps between them;
// buildScenario() turns it into a ready Scenario (with useful probes), and
// scenarioToJson() saves any scenario as a scenario file.
//
//   { "name": "Steel armour plate, 10 mm", "category": "armor",
//     "description": "...", "material": "steel_rha",
//     "shape": { "type": "plate", "thickness_mm": 10, "width_mm": 60 } }
//   { "name": "Steel ball, 10 mm", "category": "projectile",
//     "material": "steel_rha", "shape": { "type": "sphere", "diameter_mm": 10 },
//     "speed_mps": 500 }
//
// Shapes: "sphere" (diameter_mm), "rod" (length_mm, diameter_mm, optional
// nose_length_mm: a pointed conical nose that is part of the length),
// "plate" (thickness_mm, width_mm = diameter of a round plate in axisymmetric
// mode, height of the slice in plane strain).
// -----------------------------------------------------------------------------
#include <filesystem>
#include <span>
#include <string>
#include <vector>

#include <armor/Material.h>

#include "armor/ScenarioFile.h"

namespace armor {

enum class ItemCategory { Projectile, Armor };
enum class ItemShape { Sphere, Rod, Plate };

struct LibraryItem {
    std::string name;
    std::string description;
    std::string file; // file name it came from
    ItemCategory category = ItemCategory::Projectile;
    ItemShape shape = ItemShape::Sphere;
    int material = 0;        // default material (index)
    double diameter = 0.0;   // m (sphere, rod)
    double length = 0.0;     // m (rod)
    double thickness = 0.0;  // m (plate)
    double width = 0.0;      // m (plate)
    double speed = 0.0;      // m/s, suggested (projectiles)
    double noseLength = 0.0; // m (rod): conical nose, part of `length`
    // Armour standing in for a real tank's front: published estimates of its
    // protection against long rods, in mm of armour steel (RHA equivalent).
    double estimateLow = 0.0, estimateHigh = 0.0; // m; 0 = not a tank estimate
    bool isTankEstimate() const { return estimateHigh > 0.0; }
};

// Volume of a projectile item (m^3), counting a rod's conical nose.
double projectileVolume(const LibraryItem& p);
// A rod's working length for long-rod formulas: a cone counts a third of its
// length (equal volume).
double rodWorkingLength(const LibraryItem& p);

LibraryItem parseLibraryItem(const std::string& json, std::span<const Material> materials,
                             const std::string& sourceName = "item");

struct Library {
    std::vector<LibraryItem> projectiles;
    std::vector<LibraryItem> armor;
    std::vector<std::string> errors; // files that could not be read (skipped)
};

// Reads <root>/projectiles/*.json and <root>/armor/*.json (sorted by name).
Library loadLibrary(const std::filesystem::path& root, std::span<const Material> materials);

struct ArmorLayer {
    LibraryItem item;
    int material = 0;
    double thickness = 0.0; // m (starts as the item's)
    double gapBefore = 0.0; // m, empty space in front of this layer (0 for the first)
    // Glued to the next layer (only if that one has no gap): the two act as one
    // bonded body, carrying tension and shear across the joint. Real ceramic
    // armour is glued to its backing; loose, the joint reflects the impact wave
    // as tension and the tile cracks from the back almost at once.
    bool bondedToNext = false;
};

struct BuildSpec {
    LibraryItem projectile;
    int projectileMaterial = 0;
    double speed = 500.0;          // m/s
    std::vector<ArmorLayer> layers;
    double obliquity = 0.0;        // rad; 0 = straight on (axisymmetric), else plane strain
    double spacing = 0.5e-3;       // m
    double endTime = 50e-6;        // s
};

// Sensible spacing and duration for a spec (see Library.cpp for the rules).
double suggestSpacing(const BuildSpec& spec);
double suggestEndTime(const BuildSpec& spec);

// The scenario, with two probes: the projectile's tail and the back face of
// the last layer.
ScenarioFile buildScenario(const BuildSpec& spec, std::span<const Material> materials);

// Any scenario as scenario-file JSON (shapes written as rect / disc).
std::string scenarioToJson(const ScenarioFile& file, std::span<const Material> materials);

} // namespace armor
