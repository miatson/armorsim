#include "armor/BallisticLimit.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <armor/Solver.h>

namespace armor {

namespace {

struct ShotState {
    double projVx = 0.0;     // mass-weighted
    double tail = 0.0, nose = 0.0;
    double plateVx = 0.0;    // ring of plate around the projectile's path
    double plateBack = 0.0;  // current back surface of that ring
    bool havePlate = false;
};

ShotState measure(const Solver& solver, std::uint16_t projBody, double halfWidth) {
    const Particles& P = solver.particles();
    ShotState st;
    double pm = 0.0, pmv = 0.0, rm = 0.0, rmv = 0.0;
    st.tail = std::numeric_limits<double>::infinity();
    st.nose = -std::numeric_limits<double>::infinity();
    st.plateBack = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < solver.particleCount(); ++i) {
        const double ay = std::abs(P.pos[i].y);
        if (P.part[i] == projBody) {
            pm += P.mass[i];
            pmv += P.mass[i] * P.vel[i].x;
            st.tail = std::min(st.tail, P.pos[i].x);
            st.nose = std::max(st.nose, P.pos[i].x);
        } else if (ay >= 1.5 * halfWidth && ay <= 3.0 * halfWidth && !fullyFailed(P.failed[i])) {
            // The ring just outside the projectile's path stays part of the
            // plate; a punched-out plug (inside the path) is excluded.
            rm += P.mass[i];
            rmv += P.mass[i] * P.vel[i].x;
            st.plateBack = std::max(st.plateBack, P.pos[i].x);
        }
    }
    st.projVx = pm > 0 ? pmv / pm : 0.0;
    st.havePlate = rm > 0;
    st.plateVx = rm > 0 ? rmv / rm : 0.0;
    return st;
}

} // namespace

ShotResult fireShot(const Scenario& scenario, std::span<const Material> materials) {
    ShotResult r;
    r.speed = std::hypot(scenario.parts.at(0).velocity.x, scenario.parts.at(0).velocity.y);
    const auto t0 = std::chrono::steady_clock::now();
    Solver solver(scenario, std::vector<Material>(materials.begin(), materials.end()));
    const std::uint16_t projBody = bodyId(scenario, 0);
    double halfWidth = 0.0;
    for (std::size_t i = 0; i < solver.particleCount(); ++i)
        if (solver.particles().part[i] == projBody)
            halfWidth = std::max(halfWidth, std::abs(solver.particles().pos[i].y));
    halfWidth += 0.5 * scenario.dx;

    const double limit = 5.0 * std::max(r.speed, 1.0);
    bool contact = false;
    std::uint64_t steps = 0;
    ShotState st;
    while (!solver.finished()) {
        solver.step();
        if (solver.diagnostics().maxSpeed > limit) {
            r.blewUp = true;
            break;
        }
        if (++steps % 50 != 0) continue;
        st = measure(solver, projBody, halfWidth);
        if (!st.havePlate) continue;
        const double vRel = st.projVx - st.plateVx;
        if (st.projVx < 0.98 * r.speed) contact = true; // it has started to slow down
        if (!contact) continue;
        // Outcome clear: fully through the plate and still faster than it ...
        if (st.tail > st.plateBack && vRel > 1.0) break;
        // ... or no longer moving forward relative to the plate.
        if (vRel <= 0.0) break;
    }
    st = measure(solver, projBody, halfWidth);
    r.residualVelocity = st.projVx;
    r.plateVelocity = st.plateVx;
    r.perforated = !r.blewUp && st.havePlate && st.nose > st.plateBack && st.projVx - st.plateVx > 1.0;
    r.timeSimulated = solver.time();
    r.wallSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return r;
}

bool rechtIpsonFit(std::span<const ShotResult> shots, double& limit, double& a) {
    // v_r² = a² v² + c, with c = −a² v_bl²: a straight line in (v², v_r²).
    double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (const ShotResult& s : shots) {
        if (!s.perforated || s.residualVelocity <= 0.0) continue;
        const double x = s.speed * s.speed, y = s.residualVelocity * s.residualVelocity;
        n += 1;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    if (n < 2) return false;
    const double den = n * sxx - sx * sx;
    if (std::abs(den) < 1e-30) return false;
    const double slope = (n * sxy - sx * sy) / den, icpt = (sy - slope * sx) / n;
    if (slope <= 0.0 || icpt >= 0.0) return false;
    a = std::sqrt(slope);
    limit = std::sqrt(-icpt / slope);
    return true;
}

BallisticLimitResult findBallisticLimit(const std::function<Scenario(double)>& makeScenario, double low, double high,
                                        int iterations, std::span<const Material> materials,
                                        const std::function<void(const ShotResult&)>& onShot) {
    BallisticLimitResult out;
    auto fire = [&](double v) {
        ShotResult s = fireShot(makeScenario(v), materials);
        s.speed = v;
        out.shots.push_back(s);
        if (!s.blewUp) {
            if (s.perforated) out.lowestPerforated = out.lowestPerforated > 0 ? std::min(out.lowestPerforated, v) : v;
            else out.highestStopped = std::max(out.highestStopped, v);
        }
        if (onShot) onShot(s);
        return s;
    };
    ShotResult lo = fire(low), hi = fire(high);
    for (int k = 0; k < 4 && lo.perforated; ++k) {
        high = low;
        low *= 0.7;
        lo = fire(low);
    }
    for (int k = 0; k < 4 && !hi.perforated; ++k) {
        low = high;
        high *= 1.4;
        hi = fire(high);
    }
    if (out.bracketed()) {
        for (int k = 0; k < iterations; ++k) {
            const double mid = out.v50();
            fire(mid);
        }
    }
    out.hasFit = rechtIpsonFit(out.shots, out.fitLimit, out.fitA);
    return out;
}

} // namespace armor
