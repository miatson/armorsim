#pragma once
// -----------------------------------------------------------------------------
// NeighborGrid.h - find all particles within distance R of each particle.
//
// Checking every pair is O(n²): 100,000 particles = 5 billion pairs per step.
// Instead we drop particles into square cells of size R. Any neighbour of a
// particle must then be in its own cell or one of the 8 cells around it, so
// each particle checks ~9 cells × a few particles: O(n) in total.
//
// Cells are stored in a HASH TABLE rather than a big 2D array, so fragments
// flying far away don't create an enormous mostly-empty grid. Different cells
// may land in the same hash bucket; we compare the exact cell coordinates, so
// that never produces wrong or duplicate neighbours.
//
// Result: a compressed list (CSR format) - neighbors(i) gives the indices of
// all particles j ≠ i with |x_i − x_j| < R, always in the same order
// (deterministic).
// -----------------------------------------------------------------------------
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "armor/ThreadPool.h"
#include "armor/Vec2.h"

namespace armor {

class NeighborGrid {
public:
    void build(std::span<const Vec2> positions, double radius, ThreadPool& pool);

    std::span<const std::uint32_t> neighbors(std::size_t i) const {
        return {list_.data() + start_[i], start_[i + 1] - start_[i]};
    }

    std::size_t totalNeighborEntries() const { return list_.size(); }

private:
    std::vector<std::int32_t> cellX_, cellY_;
    std::vector<std::uint32_t> bucketOf_;
    std::vector<std::uint32_t> bucketStart_; // size tableSize + 1
    std::vector<std::uint32_t> sorted_;      // particle indices sorted by bucket
    std::vector<std::uint32_t> count_;
    std::vector<std::size_t> start_;         // size n + 1
    std::vector<std::uint32_t> list_;
};

} // namespace armor
