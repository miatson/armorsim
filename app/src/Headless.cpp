// -----------------------------------------------------------------------------
// Headless.cpp - run a preset without a window and print a summary.
//   armorsim --headless --scenario N
// Used to check presets from scripts (does it finish? energy? final sizes?).
// -----------------------------------------------------------------------------
#include "Headless.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

#include <armor/Recorder.h>
#include <armor/Solver.h>

#include "Presets.h"

int runHeadless(int scenarioIndex) {
    const auto& list = presets();
    if (scenarioIndex < 0 || scenarioIndex >= static_cast<int>(list.size())) {
        std::fprintf(stderr, "scenario must be 1..%zu\n", list.size());
        return 2;
    }
    const Preset& preset = list[static_cast<std::size_t>(scenarioIndex)];
    const armor::Scenario scenario = preset.build(preset.defaults);
    armor::Solver solver(scenario, armor::defaultMaterials());
    std::printf("%s\n  %zu particles, dx %.3f mm, end %.1f us\n", preset.name.c_str(), solver.particleCount(),
                scenario.dx * 1e3, scenario.endTime * 1e6);

    const armor::Diagnostics d0 = solver.diagnostics();
    const double speedLimit = 5.0 * std::max(d0.maxSpeed, 1.0); // far above anything physical here
    const auto t0 = std::chrono::steady_clock::now();
    bool blewUp = false;
    unsigned long long steps = 0;
    while (!solver.finished()) {
        solver.step();
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
                (d1.kinetic + d1.internal) / (d0.kinetic + d0.internal), d1.kinetic / (d1.kinetic + d1.internal),
                d1.maxSpeed);
    std::printf("  max plastic strain %.3f, max temperature %.0f K, max pressure now %.2f GPa\n", maxEp, maxT,
                maxP / 1e9);

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
    return blewUp ? 1 : 0;
}
