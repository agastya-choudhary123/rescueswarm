#include "rescueswarm/simulation/simulation_clock.hpp"
#include <algorithm>
namespace rs { int SimulationClock::advance(double frame,double scale,const std::function<void(float)>& tick){accumulator_+=std::min(frame,0.25)*scale;int n=0;while(accumulator_>=step_&&n<16){tick(static_cast<float>(step_));accumulator_-=step_;++n;}return n;} }

