#include <doctest/doctest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include <armor/Kernel.h>
#include <armor/NeighborGrid.h>

using namespace armor;
using doctest::Approx;

TEST_CASE("cubic spline kernel integrates to 1") {
    // Sum W over a fine square lattice times the cell area ≈ ∫W dA = 1.
    const double h = 1.0;
    const CubicSplineKernel k(h);
    const double dx = 0.05;
    double sum = 0.0;
    for (int j = -60; j <= 60; ++j) {
        for (int i = -60; i <= 60; ++i) {
            sum += k.W(std::hypot(i * dx, j * dx)) * dx * dx;
        }
    }
    CHECK(sum == Approx(1.0).epsilon(1e-3).scale(0));
    CHECK(k.W(2.0 * h) == 0.0);
    CHECK(k.W(2.5 * h) == 0.0);
}

TEST_CASE("kernel gradient matches finite differences and is antisymmetric") {
    const CubicSplineKernel k(0.7);
    for (double r : {0.1, 0.5, 0.9, 1.2, 1.35}) {
        const double eps = 1e-6;
        const double numeric = (k.W(r + eps) - k.W(r - eps)) / (2 * eps);
        CHECK(k.dWdr(r) == Approx(numeric).epsilon(1e-5));
    }
    const Vec2 d{0.3, -0.4};
    const Vec2 g1 = k.gradW(d, length(d));
    const Vec2 g2 = k.gradW(-d, length(d));
    CHECK(g1.x == Approx(-g2.x));
    CHECK(g1.y == Approx(-g2.y));
}

TEST_CASE("neighbour grid matches brute force") {
    std::vector<Vec2> pts;
    std::uint32_t s = 12345;
    auto rnd = [&] {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        return (s >> 8) / 16777216.0;
    };
    for (int i = 0; i < 3000; ++i) {
        pts.push_back({rnd() * 2.0 - 1.0, rnd() * 2.0 - 1.0});
    }
    pts.push_back({50.0, 50.0}); // a far-away fragment

    const double radius = 0.07;
    ThreadPool pool(4);
    NeighborGrid grid;
    grid.build(pts, radius, pool);

    for (std::size_t i = 0; i < pts.size(); i += 37) {
        std::vector<std::uint32_t> brute;
        for (std::size_t j = 0; j < pts.size(); ++j) {
            if (j != i && lengthSquared(pts[i] - pts[j]) < radius * radius) {
                brute.push_back(static_cast<std::uint32_t>(j));
            }
        }
        auto got = grid.neighbors(i);
        std::vector<std::uint32_t> sorted(got.begin(), got.end());
        std::sort(sorted.begin(), sorted.end());
        CHECK(sorted == brute);
    }
    CHECK(grid.neighbors(pts.size() - 1).empty());
}
