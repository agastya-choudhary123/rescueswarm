#pragma once
#include "rescueswarm/simulation/world.hpp"
#include <iosfwd>
namespace rs {
struct AggregateMetrics {
    int missions = 0, completed = 0, survivors_found = 0, survivors_total = 0, collisions = 0;
    int gps_losses = 0, gps_recoveries = 0;
    double coverage_sum = 0, time_sum = 0;
    void add(const MissionMetrics& m);
    void print(std::ostream& out) const;
};
}

