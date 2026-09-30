#pragma once
#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace rs {
using Vec3 = Eigen::Vector3f;
using Cell = Eigen::Vector3i;
enum class Occupancy : unsigned char { Unknown, Free, Occupied };

class VoxelGrid {
public:
    VoxelGrid(Cell dimensions = {40, 12, 40}, float resolution = 1.0f);
    bool valid(const Cell& c) const;
    Occupancy at(const Cell& c) const;
    void set(const Cell& c, Occupancy value);
    Cell worldToCell(const Vec3& p) const;
    Vec3 cellCenter(const Cell& c) const;
    std::vector<Cell> neighbors(const Cell& c) const;
    const Cell& dimensions() const { return dimensions_; }
    float resolution() const { return resolution_; }
    std::size_t count(Occupancy value) const;
private:
    std::size_t index(const Cell& c) const;
    Cell dimensions_;
    float resolution_;
    std::vector<Occupancy> cells_;
};
}

