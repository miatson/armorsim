#pragma once
// -----------------------------------------------------------------------------
// Probes.h - time histories at chosen points, like gauges in an experiment.
//
// A probe is LAGRANGIAN: at t = 0 it picks the particle nearest its position
// and then follows that particle (that material point) wherever it moves -
// like a manganin gauge embedded in the target, or a VISAR spot on a free
// surface that moves with it. Samples are taken every `interval` and can be
// written to CSV (one row per sample, one column per probe quantity).
// -----------------------------------------------------------------------------
#include <cstddef>
#include <ostream>
#include <string>
#include <vector>

#include <armor/Particles.h>
#include <armor/Vec2.h>

namespace armor {

enum class ProbeQuantity {
    VelocityX, VelocityY, Speed, Pressure, VonMises, StressXX, StressYY, ShearXY,
    Density, PlasticStrain, Temperature, Damage, Porosity, PositionX, PositionY
};

// Name used in files and CSV headers ("velocity_x", ...), and its unit.
const char* probeQuantityName(ProbeQuantity q);
const char* probeQuantityUnit(ProbeQuantity q);
bool parseProbeQuantity(const std::string& name, ProbeQuantity& out);
std::string probeQuantityList(); // all names, comma-separated (for error messages)

struct ProbeSpec {
    std::string name;
    Vec2 position; // m, where to pick the particle at t = 0
    std::vector<ProbeQuantity> quantities;
};

class Solver;

class ProbeRecorder {
public:
    // Picks each probe's particle (nearest to its position) from the solver's
    // current state - call before the first step.
    ProbeRecorder(std::vector<ProbeSpec> probes, const Solver& solver, double interval);

    // Call after every solver step; records when the next sample time is due.
    void update(const Solver& solver);

    const std::vector<ProbeSpec>& probes() const { return probes_; }
    std::size_t particleOf(std::size_t probe) const { return particle_[probe]; }
    // Distance from the requested position to the picked particle, m.
    double pickError(std::size_t probe) const { return pickError_[probe]; }

    const std::vector<double>& times() const { return times_; }
    // values()[sample][column]; columns run probe by probe, quantity by quantity.
    const std::vector<std::vector<double>>& values() const { return values_; }
    std::vector<std::string> columnNames() const; // "probe: quantity [unit]"

    void writeCsv(std::ostream& out) const;

private:
    void sample(const Solver& solver);
    std::vector<ProbeSpec> probes_;
    std::vector<std::size_t> particle_;
    std::vector<double> pickError_;
    double interval_;
    double next_ = 0.0;
    std::vector<double> times_;
    std::vector<std::vector<double>> values_;
};

double probeValue(const Particles& P, std::size_t i, ProbeQuantity q);

} // namespace armor
