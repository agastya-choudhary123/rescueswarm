#pragma once
#include "rescueswarm/planning/voxel_grid.hpp"
#include <vector>
namespace rs {
struct PathResult { std::vector<Cell> cells; int expanded = 0; bool found = false; };
PathResult astar3D(const VoxelGrid& grid, const Cell& start, const Cell& goal, bool unknown_is_free = true);
}

