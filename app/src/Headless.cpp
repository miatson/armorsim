// -----------------------------------------------------------------------------
// Headless.cpp - run a scenario without a window and print a summary.
//   miataphysim --headless --scenario N                 built-in preset N
//   miataphysim --headless --file x.json [--csv out.csv] scenario file, probes to CSV
// Used to check scenarios from scripts (does it finish? energy? final sizes?).
// -----------------------------------------------------------------------------
#include "Headless.h"

#include <algorithm>
#include <chrono>
#include <string>
#include <cmath>
#include <cstdio>
#include <vector>

#include <fstream>

#include <armor/BallisticLimit.h>
#include <armor/Recorder.h>
#include <armor/ScenarioFile.h>
#include <armor/Solver.h>

#include "Presets.h"

namespace {

int runScenario(const std::string& title, const armor::Scenario& scenario, const std::vector<armor::ProbeSpec>& probes,
                double probeInterval, const std::string& csvPath) {
    armor::Solver solver(scenario, armor::defaultMaterials());
    std::printf("%s\n  %zu particles, dx %.3f mm, end %.1f us\n", title.c_str(), solver.particleCount(),
                scenario.dx * 1e3, scenario.endTime * 1e6);
    armor::ProbeRecorder recorder(probes, solver, probeInterval > 0 ? probeInterval : scenario.endTime / 1000.0);
    for (std::size_t k = 0; k < probes.size(); ++k) {
        const armor::Vec2 at = solver.particles().pos[recorder.particleOf(k)];
        std::printf("  probe \"%s\" follows the particle at (%.3f, %.3f) mm (%.3f mm from the requested point)\n",
                    probes[k].name.c_str(), at.x * 1e3, at.y * 1e3, recorder.pickError(k) * 1e3);
    }

    const armor::Diagnostics d0 = solver.diagnostics();
    const double speedLimit = 5.0 * std::max(d0.maxSpeed, 1.0); // far above anything physical here
    const auto t0 = std::chrono::steady_clock::now();
    bool blewUp = false;
    unsigned long long steps = 0;
    while (!solver.finished()) {
        solver.step();
        recorder.update(solver);
        ++steps;
        if (solver.diagnostics().maxSpeed > speedLimit) {
            blewUp = true;
            break;
        }
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const armor::Diagnostics d1 = solver.diagnostics();
    const armor::Particles& P = solver.particles();

    double maxEp = 0, maxT = 0, maxP = 0;
    for (std::size_t i = 0; i < solver.particleCount(); ++i) {
        maxEp = std::max(maxEp, P.plasticStrain[i]);
        maxT = std::max(maxT, P.temperature[i]);
        maxP = std::max(maxP, P.pressure[i]);
    }
    std::printf("  %s at t = %.2f us after %llu steps, %.1f s wall\n", blewUp ? "BLEW UP" : "finished",
                solver.time() * 1e6, steps, wall);
    std::printf("  energy ratio %.4f, kinetic share %.3f, max speed %.1f m/s\n",
                (d1.kinetic + d1.internal + d1.absorbed) / (d0.kinetic + d0.internal), d1.kinetic / (d1.kinetic + d1.internal),
                d1.maxSpeed);
    if (solver.heldParticleCount() > 0 || d1.absorbed > 0.0)
        std::printf("  mounts: %zu particles still held, %.3g J absorbed (%.1f%% of the start energy)\n",
                    solver.heldParticleCount(), d1.absorbed, 100.0 * d1.absorbed / (d0.kinetic + d0.internal));
    if (!solver.mountNote().empty()) std::printf("  note: %s\n", solver.mountNote().c_str());
    std::printf("  max plastic strain %.3f, max temperature %.0f K, max pressure now %.2f GPa\n", maxEp, maxT,
                maxP / 1e9);

    // Fragments (Stage C): pieces of at least 3 particles, heaviest first.
    {
        const armor::FragmentStats fs = solver.fragments();
        const auto names = armor::bodyNames(scenario);
        for (const armor::BodyName& b : names) {
            int count = 0;
            std::string top;
            for (const armor::Fragment& f : fs.pieces) {
                if (f.body != b.id || f.particles < 3) continue;
                if (++count <= 3) {
                    char buf[96];
                    std::snprintf(buf, sizeof buf, "%s%.3g g at %.0f m/s", top.empty() ? "" : ", ", f.mass * 1e3,
                                  std::hypot(f.velocity.x, f.velocity.y));
                    top += buf;
                }
            }
            std::printf("  %-26s %d piece(s)%s%s\n", b.name.c_str(), count, count ? ": " : "", top.c_str());
        }
        std::printf("  loose debris (fully failed): %zu particles, %.3g g\n", fs.debrisParticles, fs.debrisMass * 1e3);
    }

    // Body sizes, measured to the surface (centres sit dx/2 inside it).
    const bool axi = scenario.mode == armor::GeometryMode::Axisymmetric;
    for (const armor::BodyName& b : armor::bodyNames(scenario)) {
        double x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
        for (std::size_t i = 0; i < solver.particleCount(); ++i) {
            if (P.part[i] != b.id) continue;
            x0 = std::min(x0, P.pos[i].x);
            x1 = std::max(x1, P.pos[i].x);
            y0 = std::min(y0, P.pos[i].y);
            y1 = std::max(y1, P.pos[i].y);
        }
        if (x1 < x0) continue;
        const double len = (x1 - x0 + scenario.dx) * 1e3;
        const double wid = axi ? 2.0 * (y1 + scenario.dx / 2) * 1e3 : (y1 - y0 + scenario.dx) * 1e3;
        std::printf("  %-26s x %.2f..%.2f mm  length %.2f mm, %s %.2f mm\n", b.name.c_str(), x0 * 1e3, x1 * 1e3,
                    len, axi ? "diameter" : "width", wid);
    }
    if (!csvPath.empty()) {
        std::ofstream out(csvPath);
        if (!out) {
            std::fprintf(stderr, "cannot write %s\n", csvPath.c_str());
            return 2;
        }
        recorder.writeCsv(out);
        std::printf("  probes: %zu samples written to %s\n", recorder.times().size(), csvPath.c_str());
    }
    return blewUp ? 1 : 0;
}

} // namespace

int runHeadless(int scenarioIndex, const std::string& csvPath) {
    const auto& list = presets();
    if (scenarioIndex < 0 || scenarioIndex >= static_cast<int>(list.size())) {
        std::fprintf(stderr, "scenario must be 1..%zu\n", list.size());
        return 2;
    }
    const Preset& preset = list[static_cast<std::size_t>(scenarioIndex)];
    return runScenario(preset.name, preset.build(preset.defaults), {}, 0.0, csvPath);
}

int runHeadlessFile(const std::string& path, const std::string& csvPath, const HeadlessOverrides& o) {
    try {
        armor::ScenarioFile file = armor::loadScenarioFile(path, armor::defaultMaterials());
        armor::Scenario& s = file.scenario;
        // Overrides, like the app's sliders: speed scales every moving part.
        if (o.speed > 0) {
            double fastest = 0.0;
            for (const armor::Part& p : s.parts) fastest = std::max(fastest, std::hypot(p.velocity.x, p.velocity.y));
            if (fastest > 0)
                for (armor::Part& p : s.parts) p.velocity = p.velocity * (o.speed / fastest);
        }
        if (o.spacing > 0) s.dx = o.spacing;
        if (o.endTime > 0) s.endTime = o.endTime;
        return runScenario(s.name, s, file.probes, file.probeInterval, csvPath);
    } catch (const armor::ScenarioFileError& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 2;
    }
}

// Ballistic limit (V50) of a scenario file: the projectile is its first part.
int runHeadlessV50(const std::string& path, double low, double high, int iterations, const HeadlessOverrides& o) {
    try {
        armor::ScenarioFile file = armor::loadScenarioFile(path, armor::defaultMaterials());
        armor::Scenario base = file.scenario;
        if (o.spacing > 0) base.dx = o.spacing;
        if (o.endTime > 0) base.endTime = o.endTime;
        double fastest = 0.0;
        for (const armor::Part& p : base.parts) fastest = std::max(fastest, std::hypot(p.velocity.x, p.velocity.y));
        if (fastest <= 0.0) {
            std::fprintf(stderr, "%s: nothing moves - the first part should be the projectile\n", path.c_str());
            return 2;
        }
        auto make = [&](double v) {
            armor::Scenario s = base;
            for (armor::Part& p : s.parts) p.velocity = p.velocity * (v / fastest);
            return s;
        };
        std::printf("Ballistic limit: %s\n  projectile: %s, spacing %.2f mm, up to %.0f us per shot\n", base.name.c_str(),
                    base.parts[0].name.c_str(), base.dx * 1e3, base.endTime * 1e6);
        std::printf("  %8s  %-11s %12s %10s %8s %8s\n", "speed", "result", "residual", "plate", "time", "wall");
        const armor::BallisticLimitResult r = armor::findBallisticLimit(
            make, low, high, iterations, armor::defaultMaterials(), [](const armor::ShotResult& s) {
                std::printf("  %6.1f m/s  %-11s %8.1f m/s %6.1f m/s %5.0f us %6.1f s\n", s.speed,
                            s.blewUp ? "BLEW UP" : (s.perforated ? "perforated" : "stopped"), s.residualVelocity,
                            s.plateVelocity, s.timeSimulated * 1e6, s.wallSeconds);
                std::fflush(stdout);
            });
        if (r.bracketed())
            std::printf("  V50 = %.1f m/s (between %.1f stopped and %.1f perforated)\n", r.v50(), r.highestStopped,
                        r.lowestPerforated);
        else
            std::printf("  not bracketed: highest stopped %.1f m/s, lowest perforated %.1f m/s\n", r.highestStopped,
                        r.lowestPerforated);
        if (r.hasFit)
            std::printf("  Recht-Ipson fit through the perforating shots: v_bl = %.1f m/s, a = %.3f\n", r.fitLimit, r.fitA);
        return r.bracketed() ? 0 : 1;
    } catch (const armor::ScenarioFileError& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 2;
    }
}
