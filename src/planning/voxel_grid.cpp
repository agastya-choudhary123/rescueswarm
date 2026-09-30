#include "rescueswarm/planning/voxel_grid.hpp"
#include <cmath>
namespace rs {
VoxelGrid::VoxelGrid(Cell d, float r) : dimensions_(d), resolution_(r), cells_(static_cast<std::size_t>(d.x()*d.y()*d.z()), Occupancy::Unknown) {}
bool VoxelGrid::valid(const Cell& c) const { return (c.array() >= 0).all() && (c.array() < dimensions_.array()).all(); }
std::size_t VoxelGrid::index(const Cell& c) const { return static_cast<std::size_t>((c.y()*dimensions_.z()+c.z())*dimensions_.x()+c.x()); }
Occupancy VoxelGrid::at(const Cell& c) const { return valid(c) ? cells_[index(c)] : Occupancy::Occupied; }
void VoxelGrid::set(const Cell& c, Occupancy v) { if(valid(c)) cells_[index(c)] = v; }
Cell VoxelGrid::worldToCell(const Vec3& p) const { return (p/resolution_).array().floor().cast<int>(); }
Vec3 VoxelGrid::cellCenter(const Cell& c) const { return (c.cast<float>().array()+0.5f).matrix()*resolution_; }
std::vector<Cell> VoxelGrid::neighbors(const Cell& c) const { static const Cell ds[]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}}; std::vector<Cell> out; for(auto& d:ds) if(valid(c+d)) out.push_back(c+d); return out; }
std::size_t VoxelGrid::count(Occupancy v) const { return static_cast<std::size_t>(std::count(cells_.begin(), cells_.end(), v)); }
}

