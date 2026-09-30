#include "rescueswarm/sensors/sensors.hpp"
namespace rs { void lidarScan(const Vec3&p,float range,const VoxelGrid&t,VoxelGrid&k){Cell c=t.worldToCell(p);int r=static_cast<int>(range/t.resolution());for(int y=-r;y<=r;++y)for(int z=-r;z<=r;++z)for(int x=-r;x<=r;++x){Cell q=c+Cell{x,y,z};if(t.valid(q)&&(t.cellCenter(q)-p).norm()<=range)k.set(q,t.at(q));}} }

