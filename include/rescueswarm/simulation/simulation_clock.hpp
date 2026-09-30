#pragma once
#include <functional>
namespace rs {
class SimulationClock {
public:
    explicit SimulationClock(double fixed_step = 1.0/30.0) : step_(fixed_step) {}
    int advance(double frame_seconds, double scale, const std::function<void(float)>& tick);
    double interpolation() const { return accumulator_ / step_; }
private:
    double step_, accumulator_ = 0.0;
};
}

