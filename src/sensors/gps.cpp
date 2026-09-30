#include "rescueswarm/sensors/sensors.hpp"
#include <random>
namespace rs { Vec3 gpsMeasurement(const Vec3&t,bool available,std::mt19937&r){if(!available)return t;std::normal_distribution<float> n(0,0.06f);return t+Vec3{n(r),n(r),n(r)};} }

