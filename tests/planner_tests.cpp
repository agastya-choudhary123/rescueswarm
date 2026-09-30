#include "rescueswarm/planning/astar_3d.hpp"
#include <catch2/catch_test_macros.hpp>
TEST_CASE("3D A-star routes around occupied voxels"){rs::VoxelGrid g({8,4,8},1);for(int y=0;y<4;++y)for(int z=0;z<8;++z)for(int x=0;x<8;++x)g.set({x,y,z},rs::Occupancy::Free);for(int z=0;z<7;++z)g.set({3,1,z},rs::Occupancy::Occupied);auto p=rs::astar3D(g,{1,1,1},{6,1,1},false);REQUIRE(p.found);REQUIRE(p.cells.front()==rs::Cell{1,1,1});REQUIRE(p.cells.back()==rs::Cell{6,1,1});for(auto&c:p.cells)REQUIRE(g.at(c)!=rs::Occupancy::Occupied);}
TEST_CASE("A-star rejects blocked goals"){rs::VoxelGrid g({4,4,4},1);g.set({2,2,2},rs::Occupancy::Occupied);REQUIRE_FALSE(rs::astar3D(g,{1,1,1},{2,2,2}).found);}

