#include "rescueswarm/autonomy/autonomy.hpp"
#include "rescueswarm/planning/voxel_grid.hpp"
#include <catch2/catch_test_macros.hpp>
TEST_CASE("low battery forces return and landing"){REQUIRE(rs::nextState(rs::DroneState::Explore,20,true,false,false,false)==rs::DroneState::ReturnToBase);REQUIRE(rs::nextState(rs::DroneState::ReturnToBase,20,true,false,true,false)==rs::DroneState::Land);}
TEST_CASE("detections trigger investigation"){REQUIRE(rs::nextState(rs::DroneState::Explore,90,true,true,false,false)==rs::DroneState::Investigate);REQUIRE(rs::nextState(rs::DroneState::Investigate,90,true,true,false,false)==rs::DroneState::ConfirmSurvivor);}
TEST_CASE("frontier search stays at operational flight altitude"){rs::VoxelGrid g({12,8,12},1.0f);for(int z=2;z<6;++z)for(int x=2;x<6;++x)g.set({x,2,z},rs::Occupancy::Free);auto frontiers=rs::frontierCells(g);REQUIRE_FALSE(frontiers.empty());for(const auto&cell:frontiers)REQUIRE(cell.y()==2);}
