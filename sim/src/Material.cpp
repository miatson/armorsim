#include "armor/Material.h"

#include <algorithm>
#include <utility>

namespace armor {

Material Material::fromYoungPoisson(std::string name, double density, double youngs, double poisson,
                                    std::string source) {
    Material m;
    m.name = std::move(name);
    m.source = std::move(source);
    m.density0 = density;
    m.bulkModulus = youngs / (3.0 * (1.0 - 2.0 * poisson));
    m.shearModulus = youngs / (2.0 * (1.0 + poisson));
    return m;
}

namespace {

// Keep 1 − sη away from zero: at η = 1/s the Hugoniot pressure is infinite
// (a material can't be compressed beyond a limiting density). Real impacts
// stay far below this; the clamp only protects against numerical blow-ups.
double clampCompression(double eta, double s) { return std::min(eta, 0.9 / s); }

} // namespace

double Material::pressure(double density, double energy) const {
    if (const auto* pe = std::get_if<PolynomialEos>(&eos)) {
        const double mu = density / density0 - 1.0;
        return mu > 0.0 ? mu * (pe->K1 + mu * (pe->K2 + mu * pe->K3)) : pe->K1 * mu;
    }
    if (const auto* mg = std::get_if<MieGruneisenEos>(&eos)) {
        const double rho0c2 = density0 * mg->c0 * mg->c0;
        const double eta = 1.0 - density0 / density;
        if (eta <= 0.0) {
            // Tension / expansion: linear part only.
            return rho0c2 * eta + mg->gamma0 * density0 * energy;
        }
        const double e = clampCompression(eta, mg->s);
        const double denom = 1.0 - mg->s * e;
        const double pH = rho0c2 * e / (denom * denom);
        const double eH = pH * e / (2.0 * density0);
        return pH + mg->gamma0 * density0 * (energy - eH);
    }
    return bulkModulus * (density / density0 - 1.0);
}

// c² = ∂p/∂ρ at constant entropy = ∂p/∂ρ|_e + (p/ρ²) ∂p/∂e|_ρ
double Material::bulkSoundSpeed(double density, double energy) const {
    if (const auto* pe = std::get_if<PolynomialEos>(&eos)) {
        const double mu = std::max(density / density0 - 1.0, 0.0);
        const double dp = pe->K1 + mu * (2.0 * pe->K2 + 3.0 * mu * pe->K3); // dp/dμ
        return std::sqrt(std::max(dp, pe->K1) / density0);
    }
    if (const auto* mg = std::get_if<MieGruneisenEos>(&eos)) {
        const double rho0c2 = density0 * mg->c0 * mg->c0;
        const double eta = 1.0 - density0 / density;
        if (eta <= 0.0) {
            // Expanded material: keep the reference sound speed (no weaker than
            // that - it's only used for the time step and viscosity).
            return mg->c0;
        }
        const double e = clampCompression(eta, mg->s);
        const double denom = 1.0 - mg->s * e;
        const double pH = rho0c2 * e / (denom * denom);
        const double dpH = rho0c2 * (1.0 + mg->s * e) / (denom * denom * denom); // dp_H/dη
        const double deH = (dpH * e + pH) / (2.0 * density0);                     // de_H/dη
        const double gr = mg->gamma0 * density0;                                  // ∂p/∂e
        const double p = pH + gr * (energy - pH * e / (2.0 * density0));
        const double dEtaDrho = density0 / (density * density);
        const double c2 = (dpH - gr * deH) * dEtaDrho + p * gr / (density * density);
        return std::sqrt(std::max(c2, mg->c0 * mg->c0));
    }
    return std::sqrt(bulkModulus / density);
}

// -----------------------------------------------------------------------------
// Material library. Room-temperature data from the open literature; heats
// and tempers of real alloys vary, so treat these as representative.
//
// Mie-Grüneisen constants: Steinberg, "Equation of state and strength
// properties of selected materials", LLNL UCRL-MA-106439 (1996), and
// LASL Shock Hugoniot Data (Marsh, 1980).
// Johnson-Cook constants: Johnson & Cook (1983, 1985) unless noted.
// -----------------------------------------------------------------------------
std::vector<Material> defaultMaterials() {
    std::vector<Material> list;

    {
        Material m;
        m.name = "Steel (RHA / 4340-class)";
        m.key = "steel_rha";
        m.source = "4340 steel, Rc 30: JC from Johnson & Cook 1985; EOS Steinberg 1996. "
                   "RHA (MIL-DTL-12560) is a similar Ni-Cr-Mo steel.";
        m.density0 = 7850.0;
        m.eos = MieGruneisenEos{4578.0, 1.33, 1.67};
        m.bulkModulus = m.density0 * 4578.0 * 4578.0;
        m.shearModulus = 80.2e9;
        m.specificHeat = 477.0;
        m.strength = JohnsonCookStrength{792e6, 510e6, 0.26, 0.014, 1.03, 1793.0};
        m.failure = JohnsonCookFailure{0.05, 3.44, -2.12, 0.002, 0.61};
        m.spallStrength = 3.0e9;
        m.source += " Fracture: JC 1985 (4340); spall 3 GPa (steels: ~1.6-3.5 GPa in plate impact; no 4340 value found).";
        list.push_back(m);
    }
    {
        Material m;
        m.name = "Aluminium 6061-T6";
        m.key = "aluminium_6061";
        m.source = "JC: Lesuer et al., LLNL 2001; EOS Steinberg 1996.";
        m.density0 = 2703.0;
        m.eos = MieGruneisenEos{5240.0, 1.40, 1.97};
        m.bulkModulus = m.density0 * 5240.0 * 5240.0;
        m.shearModulus = 27.6e9;
        m.specificHeat = 896.0;
        m.strength = JohnsonCookStrength{324e6, 114e6, 0.42, 0.002, 1.34, 925.0};
        m.failure = JohnsonCookFailure{-0.77, 1.45, -0.47, 0.0, 1.6};
        m.spallStrength = 1.1e9;
        m.source += " Fracture: Lesuer 2000 (6061-T6); spall 1.1 GPa (plate-impact data 0.8-2.3 GPa).";
        list.push_back(m);
    }
    {
        Material m;
        m.name = "Copper (OFHC)";
        m.key = "copper_ofhc";
        m.source = "JC: Johnson & Cook 1983; EOS Steinberg 1996.";
        m.density0 = 8960.0;
        m.eos = MieGruneisenEos{3940.0, 1.49, 2.02};
        m.bulkModulus = m.density0 * 3940.0 * 3940.0;
        m.shearModulus = 46.0e9;
        m.specificHeat = 383.0;
        m.strength = JohnsonCookStrength{90e6, 292e6, 0.31, 0.025, 1.09, 1356.0};
        m.failure = JohnsonCookFailure{0.54, 4.89, -3.03, 0.014, 1.12};
        m.spallStrength = 1.3e9;
        m.source += " Fracture: JC 1985 (OFHC copper); spall 1.3 GPa (copper ~1.28 GPa, square pulse).";
        list.push_back(m);
    }
    {
        Material m;
        m.name = "Tungsten heavy alloy (93W-Ni-Fe)";
        m.key = "tungsten_wha";
        m.source = "JC: Johnson & Cook 1983 (tungsten alloy); EOS: Steinberg 1996 (90W alloy, "
                   "c0 4029 m/s, s 1.237).";
        m.density0 = 17600.0;
        m.eos = MieGruneisenEos{4029.0, 1.237, 1.54};
        m.bulkModulus = m.density0 * 4029.0 * 4029.0;
        m.shearModulus = 140.0e9;
        m.specificHeat = 134.0;
        m.strength = JohnsonCookStrength{1506e6, 177e6, 0.12, 0.016, 1.0, 1723.0};
        m.failure = JohnsonCookFailure{0.0, 0.33, -1.5, 0.0, 0.0};
        // Spall: plate-impact measurements on tungsten heavy alloys give 1.7-2.6 GPa
        // (was 3.5 GPa, ~75% high; changed 2026-11-28 - the long-rod validation
        // in the README was run with 3.5 GPa; little effect expected, the rod is
        // mostly in compression).
        m.spallStrength = 2.0e9;
        m.source += " Fracture: JC 1985 (tungsten alloy); spall 2.0 GPa (plate-impact data 1.7-2.6 GPa).";
        list.push_back(m);
    }
    {
        Material m;
        m.name = "Titanium Ti-6Al-4V";
        m.key = "titanium_6al4v";
        m.source = "JC: Lesuer, LLNL 2000; EOS Steinberg 1996.";
        m.density0 = 4430.0;
        m.eos = MieGruneisenEos{5130.0, 1.028, 1.23};
        m.bulkModulus = m.density0 * 5130.0 * 5130.0;
        m.shearModulus = 41.9e9;
        m.specificHeat = 580.0;
        m.strength = JohnsonCookStrength{1098e6, 1092e6, 0.93, 0.014, 1.1, 1878.0};
        m.failure = JohnsonCookFailure{-0.09, 0.25, -0.5, 0.014, 3.87};
        m.spallStrength = 4.0e9;
        m.source += " Fracture: Lesuer 2000 (Ti-6Al-4V); spall 4 GPa (plate-impact data 3.3-4.9 GPa).";
        list.push_back(m);
    }
    {
        // Structural steel used in the Borvik et al. perforation benchmarks.
        Material m;
        m.name = "Steel Weldox 460 E";
        m.key = "weldox_460e";
        m.source = "JC strength and fracture: Borvik, Hopperstad, Berstad, Langseth, Eur. J. Mech. A/Solids 20 "
                   "(2001); E 200 GPa, nu 0.33. EOS as for the RHA steel (Steinberg 1996). Spall ~3 GPa (approx.).";
        m.density0 = 7850.0;
        m.eos = MieGruneisenEos{4578.0, 1.33, 1.67};
        m.bulkModulus = m.density0 * 4578.0 * 4578.0;
        m.shearModulus = 75.2e9;
        m.specificHeat = 452.0;
        JohnsonCookStrength jc{490e6, 807e6, 0.73, 0.0114, 0.94, 1800.0};
        jc.refStrainRate = 5e-4; // the fit's reference rate
        m.strength = jc;
        JohnsonCookFailure jf{0.0705, 1.732, -0.54, -0.015, 0.0};
        jf.refStrainRate = 5e-4;
        m.failure = jf;
        m.spallStrength = 3.0e9;
        list.push_back(m);
    }
    {
        // Hardened projectile steel of the Borvik et al. tests: barely deforms.
        Material m;
        m.name = "Hardened tool steel (Arne, HRC 53)";
        m.key = "tool_steel_hrc53";
        m.source = "Borvik et al., Int. J. Impact Eng. 27 (2002): projectile modelled as bilinear elastic-plastic, "
                   "yield 1900 MPa, tangent modulus 15 GPa, E 204 GPa, nu 0.33. EOS as for the RHA steel. "
                   "No fracture model (the projectiles stayed intact).";
        m.density0 = 7850.0;
        m.eos = MieGruneisenEos{4578.0, 1.33, 1.67};
        m.bulkModulus = m.density0 * 4578.0 * 4578.0;
        m.shearModulus = 76.7e9;
        m.specificHeat = 452.0;
        m.strength = JohnsonCookStrength{1900e6, 15e9, 1.0, 0.0, 1.0, 1800.0};
        m.spallStrength = 4.0e9;
        list.push_back(m);
    }
    {
        // Ceramic armour: alumina 99.5% (Coors AD-995).
        Material m;
        m.name = "Alumina ceramic (AD-995)";
        m.key = "alumina_ad995";
        // The earlier set (rho 3700, G 90 GPa, HEL 2.79 GPa) is a much weaker,
        // lower-purity alumina: real AD-995 has HEL ~6.6 GPa. With it a ceramic
        // face did no better than aluminium of equal weight.
        m.source = "JH-2 for AD-995 (Zhang et al. 2019, Compos. Struct. 111258, Table 6): rho 3890, G 152 GPa, "
                   "K1/K2/K3 231/-160/2774 GPa, HEL 6.57 GPa, A 0.88, N 0.64, B 0.28, M 0.6, T 0.312 GPa, D1 0.01, "
                   "D2 0.7, beta 1. PHEL derived from these (test). C 0.007 (the paper prints 0.07; 0.007 is the usual "
                   "value and the cautious one). No fractured-strength cap is given, so none is used.";
        m.density0 = 3890.0;
        m.eos = PolynomialEos{231e9, -160e9, 2774e9};
        m.bulkModulus = 231e9;
        m.shearModulus = 152e9;
        m.specificHeat = 880.0;
        JohnsonHolmquist2 jh;
        jh.A = 0.88; jh.B = 0.28; jh.C = 0.007; jh.M = 0.6; jh.N = 0.64;
        jh.T = 0.312e9; jh.HEL = 6.57e9; jh.PHEL = 3.51e9; jh.sigmaHEL = 1.5 * (jh.HEL - jh.PHEL);
        jh.fracturedMax = 1e9; jh.D1 = 0.01; jh.D2 = 0.7; jh.beta = 1.0;
        m.strength = jh;
        list.push_back(m);
    }
    {
        // Ceramic armour: boron carbide, very hard and light.
        Material m;
        m.name = "Boron carbide ceramic (B4C)";
        m.key = "boron_carbide";
        m.source = "JH-2 (Johnson & Holmquist 1999, J. Appl. Phys. 85): A 0.927, B 0.7, C 0.005, M 0.85, N 0.67, "
                   "T 0.26 GPa, HEL 19 GPa, fractured cap 0.2, D1 0.001, D2 0.5, K1 233, K2 -593, K3 2800 GPa, "
                   "G 197 GPa, beta 1 (cross-checked: these give PHEL = 8.71 GPa, the published value).";
        m.density0 = 2510.0;
        m.eos = PolynomialEos{233e9, -593e9, 2800e9};
        m.bulkModulus = 233e9;
        m.shearModulus = 197e9;
        m.specificHeat = 950.0;
        JohnsonHolmquist2 jh;
        jh.A = 0.927; jh.B = 0.7; jh.C = 0.005; jh.M = 0.85; jh.N = 0.67;
        jh.T = 0.26e9; jh.HEL = 19e9; jh.PHEL = 8.71e9; jh.sigmaHEL = 1.5 * (19e9 - 8.71e9);
        jh.fracturedMax = 0.2; jh.D1 = 0.001; jh.D2 = 0.5; jh.beta = 1.0;
        m.strength = jh;
        list.push_back(m);
    }
    {
        // Ceramic armour: silicon carbide (SiC-B, Holmquist & Johnson).
        Material m;
        m.name = "Silicon carbide ceramic (SiC)";
        m.key = "silicon_carbide";
        m.source = "JH-2 SiC-B (Holmquist & Johnson): rho 3215, G 193 GPa, K1 220, K2 361, K3 0 GPa, HEL 11.7 GPa, "
                   "A 0.96, N 0.65, B 0.35, M 1.0, C 0.009 (two sources agree). PHEL 5.58 GPa derived from these "
                   "(uniaxial strain), so the set is self-consistent. Tensile strength 0.37 GPa, D1 = D2 = 0.48 and "
                   "rubble cap 0.8: single-source / approximate.";
        m.density0 = 3215.0;
        m.eos = PolynomialEos{220e9, 361e9, 0.0};
        m.bulkModulus = 220e9;
        m.shearModulus = 193e9;
        m.specificHeat = 750.0;
        JohnsonHolmquist2 jh;
        jh.A = 0.96; jh.B = 0.35; jh.C = 0.009; jh.M = 1.0; jh.N = 0.65;
        jh.T = 0.37e9; jh.HEL = 11.7e9; jh.PHEL = 5.58e9; jh.sigmaHEL = 1.5 * (11.7e9 - 5.58e9);
        jh.fracturedMax = 0.8; jh.D1 = 0.48; jh.D2 = 0.48; jh.beta = 1.0;
        m.strength = jh;
        list.push_back(m);
    }
    return list;
}

int findMaterial(std::span<const Material> materials, std::string_view name) {
    for (std::size_t i = 0; i < materials.size(); ++i) {
        if (materials[i].name == name || (!materials[i].key.empty() && materials[i].key == name)) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

} // namespace armor
