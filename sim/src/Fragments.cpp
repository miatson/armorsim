#include "armor/Fragments.h"

#include <algorithm>
#include <numeric>

#include "armor/NeighborGrid.h"
#include "armor/Particles.h"
#include "armor/ThreadPool.h"

namespace armor {

namespace {

// Union-find with path halving; the smaller index becomes the root, which
// keeps the result independent of the order pairs are visited (deterministic).
std::size_t findRoot(std::vector<std::size_t>& parent, std::size_t i) {
    while (parent[i] != i) {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    return i;
}

void unite(std::vector<std::size_t>& parent, std::size_t a, std::size_t b) {
    a = findRoot(parent, a);
    b = findRoot(parent, b);
    if (a == b) return;
    if (a < b) parent[b] = a;
    else parent[a] = b;
}

} // namespace

FragmentStats findFragments(std::span<const Vec2> pos, std::span<const Vec2> vel, std::span<const double> mass,
                            std::span<const std::uint16_t> body, std::span<const std::uint8_t> failed,
                            double linkDistance) {
    const std::size_t n = pos.size();
    FragmentStats out;
    if (n == 0) return out;

    ThreadPool pool(1);
    NeighborGrid grid;
    grid.build(pos, linkDistance, pool);

    std::vector<std::size_t> parent(n);
    std::iota(parent.begin(), parent.end(), std::size_t{0});
    for (std::size_t i = 0; i < n; ++i) {
        if (fullyFailed(failed[i])) continue;
        for (const std::uint32_t j : grid.neighbors(i)) {
            if (j > i && body[j] == body[i] && !fullyFailed(failed[j])) unite(parent, i, j);
        }
    }

    // Accumulate per root.
    std::vector<std::size_t> slot(n, static_cast<std::size_t>(-1));
    for (std::size_t i = 0; i < n; ++i) {
        if (fullyFailed(failed[i])) {
            ++out.debrisParticles;
            out.debrisMass += mass[i];
            continue;
        }
        const std::size_t r = findRoot(parent, i);
        if (slot[r] == static_cast<std::size_t>(-1)) {
            slot[r] = out.pieces.size();
            out.pieces.push_back({body[i], 0, 0.0, {}, {}});
        }
        Fragment& f = out.pieces[slot[r]];
        ++f.particles;
        f.mass += mass[i];
        f.centroid += pos[i] * mass[i];
        f.velocity += vel[i] * mass[i];
    }
    for (Fragment& f : out.pieces) {
        f.centroid = f.centroid * (1.0 / f.mass);
        f.velocity = f.velocity * (1.0 / f.mass);
    }
    std::stable_sort(out.pieces.begin(), out.pieces.end(),
                     [](const Fragment& a, const Fragment& b) { return a.mass > b.mass; });
    return out;
}

} // namespace armor
