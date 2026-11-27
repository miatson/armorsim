#pragma once
// -----------------------------------------------------------------------------
// Material.h - how a material responds to compression, shape change and heat.
//
// A material model has three parts:
//
// 1. EQUATION OF STATE (EOS): pressure p from density ρ and internal energy e.
//    - LinearEos:      p = K (ρ/ρ0 − 1)            (Stage A spring; small strains)
//    - MieGruneisenEos: shock-physics EOS for metals (see below)
//
// 2. STRENGTH MODEL: the largest shear-type stress the material can carry
//    before it flows plastically (the YIELD STRESS σ_y).
//    - ElasticStrength:     never yields (Stage A behaviour)
//    - JohnsonCookStrength: σ_y depends on plastic strain, strain rate and
//                           temperature (see below)
//
// 3. ELASTIC + THERMAL constants: shear modulus G, specific heat, etc.
//
// -----------------------------------------------------------------------------
// MIE-GRÜNEISEN EOS (Hugoniot form)
//
// When a shock wave runs through a metal, experiments find a straight-line
// relation between shock speed Us and the particle velocity behind it up:
//
//     Us = c0 + s · up               (c0 ≈ bulk sound speed, s ≈ 1.2-1.6)
//
// Conservation of mass and momentum across the shock then fix the pressure
// on the "Hugoniot" curve (all states a shock can reach). With the
// compression η = 1 − ρ0/ρ:
//
//     p_H(η) = ρ0 c0² η / (1 − s η)²
//     e_H(η) = p_H η / (2 ρ0)           (energy deposited by the shock)
//
// For states off the Hugoniot (different energy at the same density) the
// Grüneisen parameter Γ says how pressure rises with thermal energy:
//
//     p = p_H(η) + Γ0 ρ0 (e − e_H(η))   (using Γρ ≈ Γ0ρ0)
//
// Small compressions give p ≈ ρ0 c0² η: the same as a spring with bulk
// modulus K = ρ0 c0². Stronger shocks make the (1 − sη)² denominator matter:
// for steel compressed to 1.25× its density (η = 0.2, a ~1.2 km/s particle
// velocity) the Hugoniot pressure is ~1.5× the linear-law value. Metals get
// stiffer the harder you squeeze them.
// In tension (η < 0) we use the linear part: p = ρ0 c0² η + Γ0 ρ0 e.
//
// -----------------------------------------------------------------------------
// JOHNSON-COOK STRENGTH (Johnson & Cook, 1983)
//
//     σ_y = (A + B εp^n) · (1 + C ln ε̇*) · (1 − T*^m)
//
//   A + B εp^n   STRAIN HARDENING: the more a metal has been plastically
//                deformed (εp = equivalent plastic strain), the stronger it
//                gets (dislocations tangle). A = initial yield stress.
//   1 + C ln ε̇*  STRAIN-RATE HARDENING: metals are stronger when deformed
//                fast. ε̇* = ε̇ / (1 /s), never below 1.
//   1 − T*^m     THERMAL SOFTENING: T* = (T − T_room)/(T_melt − T_room).
//                At the melting point the strength vanishes (it's a liquid).
//
// The constants are fitted to lab tests (tension, torsion, Hopkinson bar,
// Taylor impact) and are published for many armor and penetrator metals.
// -----------------------------------------------------------------------------
#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <optional>
#include <variant>
#include <vector>

namespace armor {

inline constexpr double kRoomTemperature = 293.15; // K (20 °C)

struct LinearEos {};

struct MieGruneisenEos {
    double c0 = 0.0;     // m/s, intercept of the Us-up line
    double s = 0.0;      // slope of the Us-up line
    double gamma0 = 0.0; // Grüneisen parameter
};

using Eos = std::variant<LinearEos, MieGruneisenEos>;

struct ElasticStrength {};

struct JohnsonCookStrength {
    double A = 0.0, B = 0.0, n = 1.0; // Pa, Pa, -
    double C = 0.0;                   // strain-rate sensitivity
    double m = 1.0;                   // thermal softening exponent
    double meltTemperature = 1000.0;  // K
    double refTemperature = kRoomTemperature;
    double refStrainRate = 1.0;       // 1/s

    double yield(double plasticStrain, double strainRate, double temperature) const {
        const double hardening = A + B * std::pow(std::max(plasticStrain, 0.0), n);
        const double rate = 1.0 + C * std::log(std::max(strainRate / refStrainRate, 1.0));
        double tStar = (temperature - refTemperature) / (meltTemperature - refTemperature);
        tStar = std::clamp(tStar, 0.0, 1.0);
        const double thermal = 1.0 - std::pow(tStar, m);
        return hardening * rate * thermal;
    }
};

using Strength = std::variant<ElasticStrength, JohnsonCookStrength>;

// Johnson-Cook FRACTURE model (Johnson & Cook 1985). Damage grows with plastic
// strain, D += Δεp / εf, and the material fails at D = 1. The strain a metal
// survives before fracture, εf, depends on
//   - stress TRIAXIALITY σ* = −p / σ_vm: pulled apart (σ* > 0) metals fail
//     after little strain, squeezed (σ* < 0) they can flow almost forever;
//   - strain rate (D4) and temperature (D5):
//   εf = [D1 + D2 exp(D3 σ*)] · [1 + D4 ln ε̇*] · [1 + D5 T*]
struct JohnsonCookFailure {
    double D1 = 0.0, D2 = 0.0, D3 = 0.0, D4 = 0.0, D5 = 0.0;
    double refStrainRate = 1.0; // 1/s

    double fractureStrain(double triaxiality, double strainRate, double tStar) const {
        // The fits cover roughly −1.5 ≤ σ* ≤ 1.5; beyond that, hold the end value.
        const double t = std::clamp(triaxiality, -1.5, 1.5);
        const double base = D1 + D2 * std::exp(D3 * t);
        const double rate = 1.0 + D4 * std::log(std::max(strainRate / refStrainRate, 1.0));
        const double thermal = 1.0 + D5 * std::clamp(tStar, 0.0, 1.0);
        // Some fits go to zero or below at high triaxiality (D1 < 0). Keep a
        // small positive floor; real spall at such stresses is handled by the
        // spall strength (Material::spallStrength) anyway.
        return std::max(base * rate * thermal, 0.01);
    }
};

struct Material {
    std::string name;
    std::string key;             // short id used in scenario files, e.g. "copper_ofhc"
    std::string source;          // where the numbers come from
    double density0 = 0.0;       // kg/m³ at room temperature, zero pressure
    double bulkModulus = 0.0;    // K, Pa (= ρ0 c0² for Mie-Grüneisen materials)
    double shearModulus = 0.0;   // G, Pa
    double specificHeat = 450.0; // J/(kg·K)
    double taylorQuinney = 0.9;  // share of plastic work turned into heat
    Eos eos = LinearEos{};
    Strength strength = ElasticStrength{};
    // Stage C: failure. No failure model = never breaks.
    std::optional<JohnsonCookFailure> failure;
    // Spall strength, Pa: tension (negative pressure) beyond this cracks the
    // material at once. Measured in plate-impact tests (e.g. Kanel et al.).
    double spallStrength = std::numeric_limits<double>::infinity();

    static Material fromYoungPoisson(std::string name, double density, double youngs, double poisson,
                                     std::string source);

    double youngsModulus() const { return 9.0 * bulkModulus * shearModulus / (3.0 * bulkModulus + shearModulus); }
    double poissonRatio() const {
        return (3.0 * bulkModulus - 2.0 * shearModulus) / (2.0 * (3.0 * bulkModulus + shearModulus));
    }

    // p(ρ, e), Pa. e = specific internal energy relative to room conditions, J/kg.
    double pressure(double density, double energy) const;

    // Bulk sound speed (speed of a pressure wave in a fluid with this EOS).
    double bulkSoundSpeed(double density, double energy) const;

    // Fastest elastic wave ("longitudinal" or P wave): sqrt(c_bulk² + 4G/(3ρ)).
    // Limits the time step.
    double soundSpeed(double density, double energy = 0.0) const {
        const double cb = bulkSoundSpeed(density, energy);
        return std::sqrt(cb * cb + 4.0 / 3.0 * shearModulus / density);
    }

    // Yield stress; +infinity for purely elastic materials.
    double yieldStress(double plasticStrain, double strainRate, double temperature) const {
        if (const auto* jc = std::get_if<JohnsonCookStrength>(&strength)) {
            return jc->yield(plasticStrain, strainRate, temperature);
        }
        return std::numeric_limits<double>::infinity();
    }

    // Plastic strain to fracture at this state (+infinity without a model).
    double fractureStrain(double triaxiality, double strainRate, double temperature) const {
        if (!failure) return std::numeric_limits<double>::infinity();
        double tStar = 0.0;
        if (const auto* jc = std::get_if<JohnsonCookStrength>(&strength)) {
            tStar = (temperature - jc->refTemperature) / (jc->meltTemperature - jc->refTemperature);
        }
        return failure->fractureStrain(triaxiality, strainRate, tStar);
    }

    double meltTemperature() const {
        if (const auto* jc = std::get_if<JohnsonCookStrength>(&strength)) return jc->meltTemperature;
        return std::numeric_limits<double>::infinity();
    }

    // Thin-rod and thin-plate elastic wave speeds (Stage A checks).
    double barWaveSpeed() const { return std::sqrt(youngsModulus() / density0); }
    double plateWaveSpeed() const {
        const double nu = poissonRatio();
        return std::sqrt(youngsModulus() / (density0 * (1.0 - nu * nu)));
    }
};

// Indices into defaultMaterials().
enum MaterialId : int {
    kSteelRHA = 0,
    kAluminium6061,
    kCopperOFHC,
    kTungstenHeavyAlloy,
    kTitanium64,
    kWeldox460E,
    kToolSteelHRC53,
};

std::vector<Material> defaultMaterials();

// Index of the material with this name or key, or -1.
int findMaterial(std::span<const Material> materials, std::string_view name);

} // namespace armor
