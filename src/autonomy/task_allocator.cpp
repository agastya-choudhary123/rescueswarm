#include "rescueswarm/autonomy/autonomy.hpp"
namespace rs { void allocateRegions(std::vector<Drone>& ds,int regions){for(std::size_t i=0;i<ds.size();++i)ds[i].assigned_region=static_cast<int>(i)%regions;} }

