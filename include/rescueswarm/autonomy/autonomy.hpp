#pragma once
#include "rescueswarm/simulation/drone.hpp"
#include <vector>
namespace rs {
class VoxelGrid;
DroneState nextState(DroneState current, float battery, bool has_path, bool survivor_near, bool at_base, bool obstacle_close);
std::vector<Cell> frontierCells(const VoxelGrid& known);
Cell chooseFrontier(const VoxelGrid& known, const Cell& from, int region, int region_count);
Cell chooseFrontierScored(const VoxelGrid& known, const Drone& drone, const std::vector<Drone>& team);
void allocateRegions(std::vector<Drone>& drones, int region_count);
Vec3 collisionAvoidance(const Drone& drone, const std::vector<Drone>& peers, const VoxelGrid& truth);
}
