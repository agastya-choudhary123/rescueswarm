#include "rescueswarm/simulation/world.hpp"
#include <catch2/catch_test_macros.hpp>
TEST_CASE("scenario loads and simulation is deterministic"){auto s=rs::Scenario::load(RESCUESWARM_SOURCE_DIR "/scenarios/training.json");rs::World a(s,42),b(s,42);for(int i=0;i<300;++i){a.tick(1.f/30);b.tick(1.f/30);}REQUIRE(a.drones().size()==2);REQUIRE((a.drones()[0].position-b.drones()[0].position).norm()<0.0001f);REQUIRE(a.metrics().coverage>0.0f);}

