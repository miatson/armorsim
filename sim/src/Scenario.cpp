#include "armor/Scenario.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "armor/Kernel.h" // kPi

namespace armor {

namespace {

struct Box {
    Vec2 min, max;
};

Box boundsOf(const Shape& shape) {
    if (const auto* r = std::get_if<RectShape>(&shape)) {
        Box b{{1e300, 1e300}, {-1e300, -1e300}};
        for (int sx : {-1, 1}) {
            for (int sy : {-1, 1}) {
                const Vec2 corner = r->center + rotate({sx * r->halfSize.x, sy * r->halfSize.y}, r->angle);
                b.min = {std::min(b.min.x, corner.x), std::min(b.min.y, corner.y)};
                b.max = {std::max(b.max.x, corner.x), std::max(b.max.y, corner.y)};
            }
        }
        return b;
    }
    const auto& d = std::get<DiscShape>(shape);
    return {d.center - Vec2{d.radius, d.radius}, d.center + Vec2{d.radius, d.radius}};
}

} // namespace

bool contains(const Shape& shape, Vec2 p) {
    if (const auto* r = std::get_if<RectShape>(&shape)) {
        const Vec2 local = rotate(p - r->center, -r->angle);
        return std::abs(local.x) <= r->halfSize.x && std::abs(local.y) <= r->halfSize.y;
    }
    const auto& d = std::get<DiscShape>(shape);
    return lengthSquared(p - d.center) <= d.radius * d.radius;
}

Particles buildParticles(const Scenario& scenario, std::span<const Material> materials) {
    const double dx = scenario.dx;
    const bool axi = scenario.mode == GeometryMode::Axisymmetric;
    Particles P;

    for (std::size_t partIndex = 0; partIndex < scenario.parts.size(); ++partIndex) {
        const Part& part = scenario.parts[partIndex];
        if (part.material < 0 || part.material >= static_cast<int>(materials.size())) {
            throw std::invalid_argument("part '" + part.name + "' has an invalid material index");
        }
        const Material& mat = materials[static_cast<std::size_t>(part.material)];
        const Box box = boundsOf(part.shape);

        const long long i0 = static_cast<long long>(std::floor(box.min.x / dx - 0.5));
        const long long i1 = static_cast<long long>(std::ceil(box.max.x / dx - 0.5));
        const long long j0 = static_cast<long long>(std::floor(box.min.y / dx - 0.5));
        const long long j1 = static_cast<long long>(std::ceil(box.max.y / dx - 0.5));

        for (long long j = j0; j <= j1; ++j) {
            for (long long i = i0; i <= i1; ++i) {
                const Vec2 p{(static_cast<double>(i) + 0.5) * dx, (static_cast<double>(j) + 0.5) * dx};
                if (axi && p.y <= 0.0) {
                    continue; // radius must be positive
                }
                if (!contains(part.shape, p)) {
                    continue;
                }
                bool taken = false;
                for (std::size_t earlier = 0; earlier < partIndex; ++earlier) {
                    if (contains(scenario.parts[earlier].shape, p)) {
                        taken = true;
                        break;
                    }
                }
                if (taken) {
                    continue;
                }

                const double volume = axi ? dx * dx * 2.0 * kPi * p.y // ring
                                          : dx * dx * 1.0;            // 1 m deep column
                P.pos.push_back(p);
                P.vel.push_back(part.velocity);
                P.mass.push_back(mat.density0 * volume);
                P.rho.push_back(mat.density0);
                P.energy.push_back(0.0);
                P.pressure.push_back(0.0);
                P.S.push_back({});
                P.plasticStrain.push_back(0.0);
                P.plasticRate.push_back(0.0);
                P.temperature.push_back(part.temperature);
                P.damage.push_back(0.0);
                P.failed.push_back(kIntact);
                P.spallTime.push_back(0.0);
                P.porosity.push_back(0.0);
                P.bulking.push_back(0.0);
                P.material.push_back(static_cast<std::uint16_t>(part.material));
                // "part" is really the contact body id: bonded parts share one.
                P.part.push_back(bodyId(scenario, partIndex));
            }
        }
    }
    return P;
}

} // namespace armor
