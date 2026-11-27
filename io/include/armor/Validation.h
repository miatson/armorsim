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

// Brinell hardness of the simulator's steel: 4340 at Rockwell C 30 (the
// Johnson-Cook data set it uses) is about BHN 286; RHA is typically 260-300.
inline constexpr double kSteelRHABrinell = 286.0;

} // namespace armor
