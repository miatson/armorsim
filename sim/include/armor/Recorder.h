#pragma once
// -----------------------------------------------------------------------------
// Recorder.h - snapshots of the simulation for replay.
//
// The solver thread pushes frames while the UI thread reads them. Frame slots
// are allocated up front, so the vector never reallocates; publishing a frame
// is "write the slot, then increase the counter" (with release/acquire memory
// ordering), so the reader only ever sees complete frames. No locks needed.
// -----------------------------------------------------------------------------
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "armor/Scenario.h"

namespace armor {

enum class Field : int {
    Material,
    Pressure,
    VonMises,
    PlasticStrain,
    Temperature,
    Damage,
    Speed,
    Density,
    StressXX,
    StressYY,
    ShearXY,
    Count
};
inline constexpr int kFieldCount = static_cast<int>(Field::Count);

enum class FieldKind { Category, Stress, Speed, Density, Strain, Temperature };

struct FieldInfo {
    const char* name;
    FieldKind kind;
    bool isSigned; // can be negative -> drawn with a diverging (blue-white-red) scale
    const char* help;
};

inline const FieldInfo& fieldInfo(Field f) {
    static const FieldInfo kInfo[kFieldCount] = {
        {"Material", FieldKind::Category, false, "Which material each particle is made of."},
        {"Pressure", FieldKind::Stress, true,
         "Mean compressive stress p = -(sxx+syy+szz)/3. Positive = squeezed, negative = pulled apart (tension)."},
        {"von Mises stress", FieldKind::Stress, false,
         "Equivalent shear stress sqrt(3/2 S:S). Capped by the yield stress: where it hits the cap the metal flows plastically."},
        {"Plastic strain", FieldKind::Strain, false,
         "Equivalent plastic strain: how much permanent deformation the metal has undergone. 0.1 = 10%. "
         "Shows where the material bent, bulged or flowed."},
        {"Temperature", FieldKind::Temperature, false,
         "Temperature. Rises from plastic work (90% becomes heat). Hot metal is softer (Johnson-Cook thermal softening)."},
        {"Damage", FieldKind::Strain, false,
         "Johnson-Cook damage D: 0 = intact, 1 = failed (cracked or molten: no strength, cannot be "
         "pulled, so cracks open and fragments separate). Spall voids show as at least 0.5: such "
         "material cannot be pulled apart any more but still resists shear."},
        {"Speed", FieldKind::Speed, false, "Particle speed |v|."},
        {"Density", FieldKind::Density, false, "Current density. Rises under compression."},
        {"Stress xx (axial)", FieldKind::Stress, true, "Total normal stress along x (the impact direction). Negative = compression."},
        {"Stress yy (radial/lateral)", FieldKind::Stress, true, "Total normal stress along y. Negative = compression."},
        {"Shear stress xy", FieldKind::Stress, true, "Shear stress in the x-y plane."},
    };
    return kInfo[static_cast<int>(f)];
}

struct Frame {
    double time = 0.0;
    double kineticEnergy = 0.0;  // J (per metre depth in plane strain)
    double internalEnergy = 0.0; // J
    double absorbedEnergy = 0.0; // J taken out by the mounts of held bodies
    std::vector<float> x, y;
    std::vector<std::uint16_t> body; // contact body id per particle
    std::array<std::vector<float>, kFieldCount> values;
};

struct BodyName {
    std::uint16_t id;
    std::string name;
};

// Names of the bodies in a scenario, keyed by the body id stored in particles
// (bonded parts share an id, and their names are joined).
inline std::vector<BodyName> bodyNames(const Scenario& s) {
    std::vector<BodyName> out;
    for (std::size_t i = 0; i < s.parts.size(); ++i) {
        const Part& p = s.parts[i];
        const std::uint16_t id = bodyId(s, i);
        bool merged = false;
        for (BodyName& b : out) {
            if (b.id == id) {
                b.name += " + " + p.name;
                merged = true;
            }
        }
        if (!merged) out.push_back({id, p.name});
    }
    return out;
}

class Recording {
public:
    Recording(GeometryMode mode, double dx, std::vector<std::string> materialNames, std::vector<BodyName> bodies,
              std::size_t capacity)
        : mode_(mode), dx_(dx), materialNames_(std::move(materialNames)), bodies_(std::move(bodies)),
          frames_(capacity) {}

    GeometryMode mode() const { return mode_; }
    double dx() const { return dx_; }
    const std::vector<std::string>& materialNames() const { return materialNames_; }
    const std::vector<BodyName>& bodies() const { return bodies_; }

    std::size_t size() const { return count_.load(std::memory_order_acquire); }
    std::size_t capacity() const { return frames_.size(); }
    const Frame& frame(std::size_t i) const { return *frames_[i]; }

    // Writer thread only. Returns false if full.
    bool push(std::unique_ptr<Frame> f) {
        const std::size_t n = count_.load(std::memory_order_relaxed);
        if (n >= frames_.size()) {
            return false;
        }
        frames_[n] = std::move(f);
        count_.store(n + 1, std::memory_order_release);
        return true;
    }

private:
    GeometryMode mode_;
    double dx_;
    std::vector<std::string> materialNames_;
    std::vector<BodyName> bodies_;
    std::vector<std::unique_ptr<Frame>> frames_;
    std::atomic<std::size_t> count_{0};
};

} // namespace armor
