#include "rescueswarm/autonomy/autonomy.hpp"
#include "rescueswarm/planning/voxel_grid.hpp"
#include <algorithm>
#include <limits>
namespace rs {
std::vector<Cell> frontierCells(const VoxelGrid& g){std::vector<Cell> out;auto d=g.dimensions();int y=std::clamp(2,1,d.y()-2);for(int z=1;z<d.z()-1;++z)for(int x=1;x<d.x()-1;++x){Cell c{x,y,z};if(g.at(c)!=Occupancy::Free)continue;bool horizontal_unknown=false;for(auto&n:g.neighbors(c))if(n.y()==y&&g.at(n)==Occupancy::Unknown){horizontal_unknown=true;break;}if(horizontal_unknown)out.push_back(c);}return out;}
Cell chooseFrontier(const VoxelGrid& g,const Cell& from,int region,int regions){auto fs=frontierCells(g);Cell best=from;int cost=std::numeric_limits<int>::max();for(auto&c:fs){if(regions>0&&c.x()*regions/g.dimensions().x()!=region)continue;int v=(c-from).cwiseAbs().sum();if(v<cost){cost=v;best=c;}}return best;}
Cell chooseFrontierScored(const VoxelGrid& g,const Drone& drone,const std::vector<Drone>& team){
    Cell from=g.worldToCell(drone.position),best=from;float best_score=-std::numeric_limits<float>::infinity();auto fs=frontierCells(g);int regions=std::max(1,static_cast<int>(team.size()));
    for(const auto&c:fs){float distance=static_cast<float>((c-from).cwiseAbs().sum());int gain=0;for(int dz=-2;dz<=2;++dz)for(int dx=-2;dx<=2;++dx){Cell q=c+Cell{dx,0,dz};if(g.valid(q)&&g.at(q)==Occupancy::Unknown)++gain;}float region_bonus=(c.x()*regions/g.dimensions().x()==drone.assigned_region)?18.0f:-8.0f;float separation=0.0f;for(const auto&peer:team)if(peer.id!=drone.id&&peer.target_active){float d=(peer.mission_target-g.cellCenter(c)).norm();if(d<10.0f)separation+=(10.0f-d)*2.2f;}float score=3.0f*gain-distance+region_bonus-separation;if(score>best_score){best_score=score;best=c;}}
    return best;
}
}
