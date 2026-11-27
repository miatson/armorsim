#include "armor/NeighborGrid.h"

#include <cmath>

namespace armor {

namespace {

std::uint32_t hashCell(std::int32_t cx, std::int32_t cy) {
    // Multiply by large primes and mix - spreads neighbouring cells over the table.
    return (static_cast<std::uint32_t>(cx) * 73856093u) ^ (static_cast<std::uint32_t>(cy) * 19349663u);
}

std::size_t nextPowerOfTwo(std::size_t v) {
    std::size_t p = 1;
    while (p < v) {
        p <<= 1;
    }
    return p;
}

} // namespace

void NeighborGrid::build(std::span<const Vec2> positions, double radius, ThreadPool& pool) {
    const std::size_t n = positions.size();
    const double invCell = 1.0 / radius;
    const double r2 = radius * radius;
    const std::size_t tableSize = nextPowerOfTwo(std::max<std::size_t>(64, 2 * n));
    const std::uint32_t mask = static_cast<std::uint32_t>(tableSize - 1);

    cellX_.resize(n);
    cellY_.resize(n);
    bucketOf_.resize(n);
    pool.parallelFor(n, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            cellX_[i] = static_cast<std::int32_t>(std::floor(positions[i].x * invCell));
            cellY_[i] = static_cast<std::int32_t>(std::floor(positions[i].y * invCell));
            bucketOf_[i] = hashCell(cellX_[i], cellY_[i]) & mask;
        }
    });

    // Counting sort by bucket (stable: keeps particle index order inside a bucket).
    bucketStart_.assign(tableSize + 1, 0);
    for (std::size_t i = 0; i < n; ++i) {
        ++bucketStart_[bucketOf_[i] + 1];
    }
    for (std::size_t b = 0; b < tableSize; ++b) {
        bucketStart_[b + 1] += bucketStart_[b];
    }
    sorted_.resize(n);
    count_.assign(tableSize, 0);
    for (std::size_t i = 0; i < n; ++i) {
        const std::uint32_t b = bucketOf_[i];
        sorted_[bucketStart_[b] + count_[b]++] = static_cast<std::uint32_t>(i);
    }

    // Visit the 3x3 cells around particle i and call f(j) for each neighbour.
    auto forEachNeighbor = [&](std::size_t i, auto&& f) {
        const std::int32_t cx = cellX_[i], cy = cellY_[i];
        for (std::int32_t dy = -1; dy <= 1; ++dy) {
            for (std::int32_t dx = -1; dx <= 1; ++dx) {
                const std::int32_t qx = cx + dx, qy = cy + dy;
                const std::uint32_t b = hashCell(qx, qy) & mask;
                for (std::uint32_t k = bucketStart_[b]; k < bucketStart_[b + 1]; ++k) {
                    const std::uint32_t j = sorted_[k];
                    if (j == i || cellX_[j] != qx || cellY_[j] != qy) {
                        continue; // itself, or another cell sharing the bucket
                    }
                    if (lengthSquared(positions[i] - positions[j]) < r2) {
                        f(j);
                    }
                }
            }
        }
    };

    // Pass 1: count neighbours per particle.
    start_.resize(n + 1);
    std::vector<std::uint32_t>& counts = count_;
    counts.assign(n, 0);
    pool.parallelFor(n, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            std::uint32_t c = 0;
            forEachNeighbor(i, [&](std::uint32_t) { ++c; });
            counts[i] = c;
        }
    });
    start_[0] = 0;
    for (std::size_t i = 0; i < n; ++i) {
        start_[i + 1] = start_[i] + counts[i];
    }

    // Pass 2: write them.
    list_.resize(start_[n]);
    pool.parallelFor(n, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            std::size_t w = start_[i];
            forEachNeighbor(i, [&](std::uint32_t j) { list_[w++] = j; });
        }
    });
}

} // namespace armor
