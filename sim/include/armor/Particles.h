#pragma once
// -----------------------------------------------------------------------------
// Particles.h - all particle data, "structure of arrays" style.
//
// Instead of std::vector<Particle> (each particle a struct), every field gets
// its own vector. Loops that only need positions and velocities then stream
// through tightly packed memory - much faster for 100k+ particles.
//
// MASS in the two geometry modes:
//   - Plane strain: each particle is a square column dx × dx × (1 m deep),
//     so m = ρ0 · dx² · 1 m. All energies are "per metre of depth".
//   - Axisymmetric: each particle is a RING around the symmetry axis with
//     cross-section dx × dx at radius y, so m = ρ0 · dx² · 2π y.
//     This mass is constant: a ring that moves outwards gets thinner.
// -----------------------------------------------------------------------------
#include <cstddef>
#include <algorithm>
#include <cstdint>
#include <vector>

#include "armor/SymTensor.h"
#include "armor/Vec2.h"

namespace armor {

// Why a particle failed (Particles::failed).
//  - kDuctile, kMelted: FULL failure. No shear stress, no tension; it can be
//    squeezed and pushed like sand or liquid, but nothing holds it to its
//    neighbours (pairs act like contact).
//  - kSpall: VOIDS. Sustained tension has opened voids. The particle stays
//    bonded and keeps its shear strength. Its porosity φ is tracked as its own
//    state (Solver::updatePorosity): the metal between the voids still obeys
//    the EOS, voids grow only while tension exceeds the void-growth threshold,
//    close under compression, and coalesce into a full crack (kDuctile) at the
//    critical porosity.
//    (Ductile metals: incipient spall barely weakens them in compression. A
//    copper Taylor rod at 175 m/s has such voids ~3 mm behind the impact face,
//    Gray, Maudlin & Chen, LA-UR-01-5614; making them strengthless instead let
//    the core squirt out and inflated the foot to 16.8 mm.)
enum FailureKind : std::uint8_t { kIntact = 0, kDuctile = 1, kSpall = 2, kMelted = 3 };

inline bool fullyFailed(std::uint8_t f) { return f == kDuctile || f == kMelted; }

struct Particles {
    std::vector<Vec2> pos;        // m
    std::vector<Vec2> vel;        // m/s
    std::vector<double> mass;     // kg (per metre depth in plane strain; ring mass in axisymmetric)
    std::vector<double> rho;      // density, kg/m³
    std::vector<double> energy;   // specific internal energy, J/kg
    std::vector<double> pressure; // Pa (positive = compression)
    std::vector<SymTensor> S;     // deviatoric stress, Pa
    std::vector<double> plasticStrain; // equivalent plastic strain εp (0 = never yielded)
    std::vector<double> plasticRate;   // plastic strain rate from the last step, 1/s
    std::vector<double> temperature;   // K
    std::vector<double> damage;        // Johnson-Cook damage D (fails at 1)
    std::vector<std::uint8_t> failed;  // 0 intact, else a FailureKind (Stage C)
    std::vector<double> spallTime;     // how long tension has exceeded the spall strength, s
    std::vector<double> porosity;      // void volume fraction φ of spalled material (0 = dense)
    std::vector<double> bulking;       // JH-2 bulking pressure of damaged ceramic, Pa
    std::vector<std::uint16_t> material;
    std::vector<std::uint16_t> part; // contact body id (bonded parts share one)

    std::size_t size() const { return pos.size(); }

    void resize(std::size_t n) {
        pos.resize(n);
        vel.resize(n);
        mass.resize(n);
        rho.resize(n);
        energy.resize(n);
        pressure.resize(n);
        S.resize(n);
        plasticStrain.resize(n);
        plasticRate.resize(n);
        temperature.resize(n);
        damage.resize(n);
        failed.resize(n);
        spallTime.resize(n);
        porosity.resize(n);
        bulking.resize(n);
        material.resize(n);
        part.resize(n);
    }

    // Append a copy of particle i; returns the new index.
    std::size_t pushCopy(std::size_t i) {
        pos.push_back(pos[i]);
        vel.push_back(vel[i]);
        mass.push_back(mass[i]);
        rho.push_back(rho[i]);
        energy.push_back(energy[i]);
        pressure.push_back(pressure[i]);
        S.push_back(S[i]);
        plasticStrain.push_back(plasticStrain[i]);
        plasticRate.push_back(plasticRate[i]);
        temperature.push_back(temperature[i]);
        damage.push_back(damage[i]);
        failed.push_back(failed[i]);
        spallTime.push_back(spallTime[i]);
        porosity.push_back(porosity[i]);
        bulking.push_back(bulking[i]);
        material.push_back(material[i]);
        part.push_back(part[i]);
        return pos.size() - 1;
    }
};

} // namespace armor
