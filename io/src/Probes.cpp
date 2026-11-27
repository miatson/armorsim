#include "armor/Probes.h"

#include <cmath>
#include <iomanip>
#include <limits>

#include <armor/Solver.h>

namespace armor {

namespace {
struct QuantityInfo {
    ProbeQuantity q;
    const char* name;
    const char* unit;
};
constexpr QuantityInfo kQuantities[] = {
    {ProbeQuantity::VelocityX, "velocity_x", "m/s"},  {ProbeQuantity::VelocityY, "velocity_y", "m/s"},
    {ProbeQuantity::Speed, "speed", "m/s"},           {ProbeQuantity::Pressure, "pressure", "Pa"},
    {ProbeQuantity::VonMises, "von_mises", "Pa"},     {ProbeQuantity::StressXX, "stress_xx", "Pa"},
    {ProbeQuantity::StressYY, "stress_yy", "Pa"},     {ProbeQuantity::ShearXY, "shear_xy", "Pa"},
    {ProbeQuantity::Density, "density", "kg/m3"},     {ProbeQuantity::PlasticStrain, "plastic_strain", "-"},
    {ProbeQuantity::Temperature, "temperature", "K"}, {ProbeQuantity::Damage, "damage", "-"},
    {ProbeQuantity::Porosity, "porosity", "-"},       {ProbeQuantity::PositionX, "x", "m"},
    {ProbeQuantity::PositionY, "y", "m"},
};
} // namespace

const char* probeQuantityName(ProbeQuantity q) {
    for (const auto& k : kQuantities)
        if (k.q == q) return k.name;
    return "?";
}

const char* probeQuantityUnit(ProbeQuantity q) {
    for (const auto& k : kQuantities)
        if (k.q == q) return k.unit;
    return "";
}

bool parseProbeQuantity(const std::string& name, ProbeQuantity& out) {
    for (const auto& k : kQuantities)
        if (name == k.name) {
            out = k.q;
            return true;
        }
    return false;
}

std::string probeQuantityList() {
    std::string s;
    for (const auto& k : kQuantities) s += std::string(s.empty() ? "" : ", ") + k.name;
    return s;
}

double probeValue(const Particles& P, std::size_t i, ProbeQuantity q) {
    const SymTensor& S = P.S[i];
    const double p = P.pressure[i];
    switch (q) {
    case ProbeQuantity::VelocityX: return P.vel[i].x;
    case ProbeQuantity::VelocityY: return P.vel[i].y;
    case ProbeQuantity::Speed: return length(P.vel[i]);
    case ProbeQuantity::Pressure: return p;
    case ProbeQuantity::VonMises: return vonMises(S);
    case ProbeQuantity::StressXX: return S.xx - p;
    case ProbeQuantity::StressYY: return S.yy - p;
    case ProbeQuantity::ShearXY: return S.xy;
    case ProbeQuantity::Density: return P.rho[i];
    case ProbeQuantity::PlasticStrain: return P.plasticStrain[i];
    case ProbeQuantity::Temperature: return P.temperature[i];
    case ProbeQuantity::Damage: return fullyFailed(P.failed[i]) ? 1.0 : std::min(P.damage[i], 1.0);
    case ProbeQuantity::Porosity: return P.porosity[i];
    case ProbeQuantity::PositionX: return P.pos[i].x;
    case ProbeQuantity::PositionY: return P.pos[i].y;
    }
    return 0.0;
}

ProbeRecorder::ProbeRecorder(std::vector<ProbeSpec> probes, const Solver& solver, double interval)
    : probes_(std::move(probes)), interval_(interval) {
    const Particles& P = solver.particles();
    for (const ProbeSpec& spec : probes_) {
        std::size_t best = 0;
        double bestD2 = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < solver.particleCount(); ++i) {
            const double d2 = lengthSquared(P.pos[i] - spec.position);
            if (d2 < bestD2) {
                bestD2 = d2;
                best = i;
            }
        }
        particle_.push_back(best);
        pickError_.push_back(std::sqrt(bestD2));
    }
    sample(solver); // t = 0
    next_ = interval_;
}

void ProbeRecorder::update(const Solver& solver) {
    if (solver.time() + 1e-15 >= next_ || solver.finished()) {
        sample(solver);
        while (next_ <= solver.time() + 1e-15) next_ += interval_;
    }
}

void ProbeRecorder::sample(const Solver& solver) {
    const Particles& P = solver.particles();
    std::vector<double> row;
    for (std::size_t k = 0; k < probes_.size(); ++k)
        for (ProbeQuantity q : probes_[k].quantities) row.push_back(probeValue(P, particle_[k], q));
    times_.push_back(solver.time());
    values_.push_back(std::move(row));
}

std::vector<std::string> ProbeRecorder::columnNames() const {
    std::vector<std::string> names;
    for (const ProbeSpec& spec : probes_)
        for (ProbeQuantity q : spec.quantities)
            names.push_back(spec.name + ": " + probeQuantityName(q) + " [" + probeQuantityUnit(q) + "]");
    return names;
}

void ProbeRecorder::writeCsv(std::ostream& out) const {
    // Quote headers (they contain spaces and colons); numbers in full precision.
    out << "\"time [s]\"";
    for (const std::string& n : columnNames()) out << ",\"" << n << '"';
    out << '\n' << std::setprecision(10);
    for (std::size_t s = 0; s < times_.size(); ++s) {
        out << times_[s];
        for (double v : values_[s]) out << ',' << v;
        out << '\n';
    }
}

} // namespace armor
