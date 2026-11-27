#include "Presets.h"

#include <cmath>
#include <cstdio>
#include <variant>

using namespace armor;

namespace {

std::string format(const char* fmt, double a = 0, double b = 0, double c = 0, double d = 0) {
    char buf[512];
    std::snprintf(buf, sizeof buf, fmt, a, b, c, d);
    return buf;
}

// Two identical steel bars flying at each other at ±v.
Scenario barImpact(GeometryMode mode, const PresetParams& p) {
    const double L = 0.1, R = 0.005;
    Scenario s;
    s.name = "Bar impact";
    s.mode = mode;
    s.dx = p.dxMm * 1e-3;
    s.endTime = p.endTimeUs * 1e-6;
    const bool axi = mode == GeometryMode::Axisymmetric;
    const double yc = axi ? R / 2 : 0.0, hy = axi ? R / 2 : R;
    s.parts.push_back({"Left bar", RectShape{{-L / 2, yc}, {L / 2, hy}}, kSteelRHA, {p.velocity, 0.0}});
    s.parts.push_back({"Right bar", RectShape{{L / 2, yc}, {L / 2, hy}}, kSteelRHA, {-p.velocity, 0.0}});
    return s;
}

std::string barTheory(bool axi, const PresetParams& p, const std::vector<Material>& m) {
    const Material& steel = m[kSteelRHA];
    const double c = axi ? steel.barWaveSpeed() : steel.plateWaveSpeed();
    const double sigma = steel.density0 * c * p.velocity;
    const std::string kind = axi ? "Rod wave speed c = sqrt(E/rho)" : "Plate wave speed c = sqrt(E/(rho(1-nu^2)))";
    return kind + format(" = %.0f m/s.\n"
                         "Impact stress s = rho*c*v = %.0f MPa (compression).\n"
                         "The wave reaches the far ends at L/c = %.1f us, reflects as tension,\n"
                         "and the bars separate at 2L/c = %.1f us with their velocities reversed.",
                         c, sigma / 1e6, 0.1 / c * 1e6, 0.2 / c * 1e6);
}

// Mie-Gruneisen constants of a material (all Stage B materials have them).
const MieGruneisenEos& eosOf(const Material& m) { return std::get<MieGruneisenEos>(m.eos); }

} // namespace

const std::vector<Preset>& presets() {
    static const std::vector<Preset> kPresets = {
        {"Bar impact (axisymmetric)",
         "Two 100 mm steel rods (10 mm diameter) hit end-on. Axisymmetric: these are real round rods. "
         "Watch the compression wave run along each rod, reflect from the free end as tension, and the rods "
         "fly apart. Classic 1D wave-mechanics experiment.",
         {5.0f, 0.5f, 60.0f}, 50.0f,
         [](const PresetParams& p) { return barImpact(GeometryMode::Axisymmetric, p); },
         [](const PresetParams& p, const std::vector<Material>& m) { return barTheory(true, p, m); }},

        {"Bar impact (plane strain)",
         "Same as above, but as a plane-strain slice: really two 10 mm thick plates, infinitely deep. "
         "Compare the wave speed with the rod version: a plate is stiffer than a rod because it can't "
         "expand sideways in the depth direction.",
         {5.0f, 0.5f, 60.0f}, 50.0f,
         [](const PresetParams& p) { return barImpact(GeometryMode::PlaneStrain, p); },
         [](const PresetParams& p, const std::vector<Material>& m) { return barTheory(false, p, m); }},

        {"Disc collision (plane strain)",
         "Two aluminium discs (20 mm) collide and bounce. At low speed they stay elastic and bounce back with "
         "almost all their kinetic energy. Push the speed past ~100 m/s and the contact zone yields: the discs "
         "flatten permanently and part of the energy turns into heat (view Plastic strain / Temperature).",
         {20.0f, 0.5f, 120.0f}, 100.0f,
         [](const PresetParams& p) {
             Scenario s;
             s.name = "Disc collision";
             s.mode = GeometryMode::PlaneStrain;
             s.dx = p.dxMm * 1e-3;
             s.endTime = p.endTimeUs * 1e-6;
             s.parts.push_back({"Disc A", DiscShape{{-0.0101, 0.0}, 0.01}, kAluminium6061, {p.velocity, 0.0}});
             s.parts.push_back({"Disc B", DiscShape{{0.0101, 0.0}, 0.01}, kAluminium6061, {-p.velocity, 0.0}});
             return s;
         },
         [](const PresetParams&, const std::vector<Material>&) {
             return std::string("Total energy (kinetic + internal) is conserved. Elastic: kinetic energy dips during\n"
                                "contact (stored as strain energy) and returns. Plastic: part of it stays as heat\n"
                                "and permanent deformation, so the discs leave slower.");
         }},

        {"Steel ball on aluminium plate (axisymmetric)",
         "A 10 mm steel ball hits a 10 mm thick aluminium plate head-on (the half-disc on the axis is a "
         "sphere in 3D). Watch the spherical stress waves spread through the plate and reflect from its back face.",
         {10.0f, 0.5f, 40.0f}, 50.0f,
         [](const PresetParams& p) {
             Scenario s;
             s.name = "Ball on plate";
             s.mode = GeometryMode::Axisymmetric;
             s.dx = p.dxMm * 1e-3;
             s.endTime = p.endTimeUs * 1e-6;
             s.parts.push_back({"Steel ball", DiscShape{{-0.0051, 0.0}, 0.005}, kSteelRHA, {p.velocity, 0.0}});
             s.parts.push_back({"Aluminium plate", RectShape{{0.005, 0.02}, {0.005, 0.02}}, kAluminium6061, {}});
             return s;
         },
         [](const PresetParams& p, const std::vector<Material>& m) {
             // Hertz contact of a sphere on a flat half-space (Johnson, Contact Mechanics, 1985):
             //   E* = 1 / ((1−ν1²)/E1 + (1−ν2²)/E2)
             //   contact time t = 2.87 (m² / (R E*² v))^(1/5)
             const Material& a = m[kSteelRHA];
             const Material& b = m[kAluminium6061];
             const double na = a.poissonRatio(), nb = b.poissonRatio();
             const double eStar = 1.0 / ((1 - na * na) / a.youngsModulus() + (1 - nb * nb) / b.youngsModulus());
             const double R = 0.005;
             const double mass = a.density0 * 4.0 / 3.0 * 3.14159265 * R * R * R;
             const double t = 2.87 * std::pow(mass * mass / (R * eStar * eStar * p.velocity), 0.2);
             const double cAl = b.soundSpeed(b.density0);
             return format("Hertz contact theory (sphere on half-space): contact lasts about %.1f us.\n"
                           "Compression wave in aluminium: %.0f m/s -> reaches the back face after %.2f us.\n"
                           "Above a few m/s the aluminium under the ball yields and the plate dents\n"
                           "permanently (view Plastic strain). For a high-speed shot use the 500 m/s preset.",
                           t * 1e6, cAl, 0.01 / cAl * 1e6);
         }},
        // ------------------------------- Stage B -------------------------------
        {"Taylor bar test: copper rod on a wall (axisymmetric)",
         "The classic test for metal strength under impact (Taylor 1948; Johnson & Cook 1983): an OFHC copper "
         "cylinder, 25.4 mm long and 7.62 mm across, hits a rigid wall at 190 m/s. The front mushrooms and the "
         "rod shortens. The wall is modelled by a mirror-image rod hitting it from the other side (by symmetry "
         "the middle plane never moves). Compare the final length with the experiment in the readout.",
         {190.0f, 0.254f, 80.0f}, 400.0f,
         [](const PresetParams& p) {
             const double L = 25.4e-3, R = 3.81e-3;
             Scenario s;
             s.name = "Taylor bar";
             s.mode = GeometryMode::Axisymmetric;
             s.dx = p.dxMm * 1e-3;
             s.endTime = p.endTimeUs * 1e-6;
             s.parts.push_back({"Mirror rod (= the wall)", RectShape{{-L / 2, R / 2}, {L / 2, R / 2}}, kCopperOFHC, {p.velocity, 0.0}});
             s.parts.push_back({"Copper rod", RectShape{{L / 2, R / 2}, {L / 2, R / 2}}, kCopperOFHC, {-p.velocity, 0.0}});
             return s;
         },
         [](const PresetParams& p, const std::vector<Material>&) {
             return format("Experiment at 190 m/s: final length 16.2 mm (from 25.4), foot ~13.5 mm.\n"
                           "This solver: ~17.2 mm at 0.254 mm spacing, 16.9 mm at 0.127 mm (converged).\n"
                           "The wall is at x = 0, so the rod's length is its right end ('to' under\n"
                           "Copper rod in Body sizes above). The rod stops after ~50-60 us.\n"
                           "Your speed: %.0f m/s. Faster = shorter rod and wider foot.\n"
                           "Inside: release waves focus on the axis and open spall voids ~3 mm behind\n"
                           "the face (view Damage, 0.5 = voids), as found by sectioning real rods\n"
                           "(Gray, Maudlin & Chen, Los Alamos: 175 m/s, voids ~3 mm from the face).", p.velocity);
         }},

        {"Shock wave: copper plate impact (axisymmetric)",
         "Two thick copper plates hit at +-v. A shock wave runs into each plate; behind it the copper is squeezed "
         "to a pressure set by the measured shock Hugoniot. View Pressure and hover a particle behind the shock "
         "front to read it. At 500 m/s that is ~21 GPa, about 200,000 atmospheres.",
         {500.0f, 0.25f, 3.0f}, 2000.0f,
         [](const PresetParams& p) {
             Scenario s;
             s.name = "Copper plate impact";
             s.mode = GeometryMode::Axisymmetric;
             s.dx = p.dxMm * 1e-3;
             s.endTime = p.endTimeUs * 1e-6;
             s.parts.push_back({"Flyer plate", RectShape{{-0.010, 0.0125}, {0.010, 0.0125}}, kCopperOFHC, {p.velocity, 0.0}});
             s.parts.push_back({"Target plate", RectShape{{0.010, 0.0125}, {0.010, 0.0125}}, kCopperOFHC, {-p.velocity, 0.0}});
             return s;
         },
         [](const PresetParams& p, const std::vector<Material>& m) {
             const Material& cu = m[kCopperOFHC];
             const MieGruneisenEos& eos = eosOf(cu);
             const double up = p.velocity;          // symmetric impact: the interface stops
             const double us = eos.c0 + eos.s * up; // measured linear Us-up relation
             const double pressure = cu.density0 * us * up;
             return format("Hugoniot: shock speed Us = c0 + s*up = %.0f m/s (up = v = %.0f m/s).\n"
                           "Shock pressure p = rho0*Us*up = %.1f GPa, density x%.3f.\n"
                           "Measure before ~3 us: after that, release waves from the plate edges arrive.",
                           us, up, pressure / 1e9, us / (us - up));
         }},

        {"Steel ball into aluminium plate, 500 m/s (axisymmetric)",
         "A 10 mm steel ball hits a 10 mm aluminium plate at 500 m/s, rifle-bullet territory. The ball digs a "
         "crater and the plate bulges at the back, then tears: view Damage to see where the aluminium fails "
         "(ductile fracture around the crater, spall near the back face).",
         {500.0f, 0.25f, 60.0f}, 1500.0f,
         [](const PresetParams& p) {
             Scenario s;
             s.name = "Ball into plate";
             s.mode = GeometryMode::Axisymmetric;
             s.dx = p.dxMm * 1e-3;
             s.endTime = p.endTimeUs * 1e-6;
             s.parts.push_back({"Steel ball", DiscShape{{-0.0051, 0.0}, 0.005}, kSteelRHA, {p.velocity, 0.0}});
             s.parts.push_back({"Aluminium plate", RectShape{{0.005, 0.02}, {0.005, 0.02}}, kAluminium6061, {}});
             return s;
         },
         [](const PresetParams& p, const std::vector<Material>& m) {
             const Material& steel = m[kSteelRHA];
             const double R = 0.005;
             const double mass = steel.density0 * 4.0 / 3.0 * 3.14159265 * R * R * R;
             return format("Ball mass %.1f g, kinetic energy %.0f J.\n"
                           "Watch: von Mises stress capped at yield in the plastic zone, Temperature rising\n"
                           "where the metal flows hardest, and the ball itself flattening slightly.",
                           mass * 1e3, 0.5 * mass * p.velocity * p.velocity);
         }},

        {"Tungsten long rod into steel armor (axisymmetric)",
         "A 100 mm tungsten heavy alloy rod (10 mm diameter, L/D 10) hits thick rolled homogeneous armor steel "
         "head-on: the kinetic-energy penetrator principle of modern tank guns, at reduced speed. At these "
         "pressures both metals flow like fluids at the rod tip; failed material from the rod and the crater "
         "wall is pushed aside and sprayed back out of the crater (view Damage).",
         {1500.0f, 0.5f, 150.0f}, 2500.0f,
         [](const PresetParams& p) {
             const double L = 0.1, R = 0.005;
             Scenario s;
             s.name = "Long rod penetration";
             s.mode = GeometryMode::Axisymmetric;
             s.dx = p.dxMm * 1e-3;
             s.endTime = p.endTimeUs * 1e-6;
             s.parts.push_back({"Tungsten rod", RectShape{{-L / 2 - 0.0005, R / 2}, {L / 2, R / 2}}, kTungstenHeavyAlloy, {p.velocity, 0.0}});
             s.parts.push_back({"Steel armor", RectShape{{0.06, 0.03}, {0.06, 0.03}}, kSteelRHA, {}});
             return s;
         },
         [](const PresetParams& p, const std::vector<Material>& m) {
             const double rp = m[kTungstenHeavyAlloy].density0, rt = m[kSteelRHA].density0;
             const double u = p.velocity / (1.0 + std::sqrt(rt / rp)); // Bernoulli at the tip, no strength
             return format("Hydrodynamic theory (no strength): tip moves at u = v/(1+sqrt(rho_t/rho_p)) = %.0f m/s,\n"
                           "maximum depth = L*sqrt(rho_p/rho_t) = %.0f mm.\n"
                           "Real tests (Lanz-Odermatt) give less, about 0.8-0.9 L at 1500 m/s, because the\n"
                           "armor's strength resists. Quantitative validation comes in Stage E.",
                           u, 100.0 * std::sqrt(rp / rt));
         }},
    };
    return kPresets;
}
