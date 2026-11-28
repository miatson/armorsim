#pragma once
// -----------------------------------------------------------------------------
// Validation.h - empirical formulas to compare simulations against (Stage E).
//
// LANZ-ODERMATT long-rod penetration (Lanz & Odermatt 1992; coefficients as
// used in Willi Odermatt's calculator, longrods.ch, fitted to 148 tungsten
// penetration tests into semi-infinite RHA, standard deviation 3.9%):
//
//   P / L = a · 1/tanh(b0 + b1·L/D) · sqrt(rho_p/rho_t) · exp(−s²/v²)
//   s²    = (c0 + c1·BHN) · BHN / rho_p
//
//   a = 0.921, b0 = 0.283, b1 = 0.0656, c0 = 138, c1 = −0.10   (tungsten -> RHA)
//
// Units (not stated by the sources; fixed by plugging in numbers - the only
// combination that gives the well-known P/L ≈ 0.85 for an L/D 10 tungsten rod
// at 1.5 km/s): densities in kg/m³, velocity in km/s, BHN = Brinell hardness.
// Flat-nosed rods without fins: working length = L. Normal impact only.
// -----------------------------------------------------------------------------
#include <cmath>

namespace armor {

// Depth of penetration divided by rod length, for a tungsten rod into steel
// armour of Brinell hardness `targetBHN`. v in m/s, densities in kg/m³.
inline double lanzOdermattTungstenRHA(double lengthOverDiameter, double rhoProjectile, double rhoTarget,
                                      double targetBHN, double velocity) {
    const double a = 0.921, b0 = 0.283, b1 = 0.0656, c0 = 138.0, c1 = -0.10;
    const double vKmS = velocity / 1000.0;
    const double s2 = (c0 + c1 * targetBHN) * targetBHN / rhoProjectile;
    return a / std::tanh(b0 + b1 * lengthOverDiameter) * std::sqrt(rhoProjectile / rhoTarget) *
           std::exp(-s2 / (vKmS * vKmS));
}

// ODERMATT PERFORATION EQUATION (longrods.ch, 2022): the speed-thickness
// relation for PERFORATING a finite (possibly angled) steel plate - the basis
// of the penetration figures usually quoted for tank rounds and of RHA-
// equivalent armour estimates. Same form as above, plus obliquity:
//
//   P / Lw = a * 1/tanh(b0 + b1*Lw/D) * cos(theta)^m * sqrt(rho_p/rho_t) * exp(-s^2/v^2)
//   s^2    = (c0 + c1*BHN_t) * BHN_t / rho_p
//
//   b0 = 0.283, b1 = 0.0656, m = -0.224
//   tungsten: a 0.994, c0 134.5, c1 -0.148   (99 tests, sd 3.3%)
//   DU:       a 0.825, c0 90.0,  c1 -0.0849  (42 tests, sd 3.5%)
//
// P is the plate thickness along the shot line that is just perforated. Valid
// for Lw/D >= 4, obliquity 0-75 deg, target 150-500 BHN. Lw = working length
// (penetrator without tracer; a conical tip counts partly - see Library.h).
enum class PenetratorKind { Tungsten, DepletedUranium };

inline double odermattPerforation(double workingLengthOverDiameter, double rhoProjectile, double rhoTarget,
                                  double targetBHN, double velocity, double obliquity, PenetratorKind kind) {
    const bool du = kind == PenetratorKind::DepletedUranium;
    const double a = du ? 0.825 : 0.994, c0 = du ? 90.0 : 134.5, c1 = du ? -0.0849 : -0.148;
    const double b0 = 0.283, b1 = 0.0656, m = -0.224;
    const double vKmS = velocity / 1000.0;
    const double s2 = (c0 + c1 * targetBHN) * targetBHN / rhoProjectile;
    return a / std::tanh(b0 + b1 * workingLengthOverDiameter) * std::pow(std::cos(obliquity), m) *
           std::sqrt(rhoProjectile / rhoTarget) * std::exp(-s2 / (vKmS * vKmS));
}

// Thick rolled armour (above ~100 mm) is softer than thin plate; roughly
// 240-280 BHN. 260 is used for the tank check.
inline constexpr double kThickRHABrinell = 260.0;
// Depleted uranium alloy (U-0.75Ti) density, for DU rounds in the formulas.
inline constexpr double kDepletedUraniumDensity = 18600.0;

// Long rods slow down in flight; published APFSDS figures suggest roughly
// 50-70 m/s per km. Used for the tank check's range setting (an assumption,
// not a measured value for any particular round).
inline constexpr double kRodSpeedLossPerKm = 60.0;

// Brinell hardness of the simulator's steel: 4340 at Rockwell C 30 (the
// Johnson-Cook data set it uses) is about BHN 286; RHA is typically 260-300.
inline constexpr double kSteelRHABrinell = 286.0;

} // namespace armor
