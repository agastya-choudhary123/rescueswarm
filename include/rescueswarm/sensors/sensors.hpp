#pragma once
#include "rescueswarm/planning/voxel_grid.hpp"
#include <random>
namespace rs {
class World;
void lidarScan(const Vec3& position, float range, const VoxelGrid& truth, VoxelGrid& known);
Vec3 gpsMeasurement(const Vec3& truth, bool available, std::mt19937& rng);
bool survivorDetected(const Vec3& drone, const Vec3& survivor, float range, bool sensor_available, std::mt19937& rng);
}

