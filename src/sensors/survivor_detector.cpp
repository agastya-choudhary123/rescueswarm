#include "rescueswarm/sensors/sensors.hpp"
namespace rs { bool survivorDetected(const Vec3&d,const Vec3&s,float range,bool ok,std::mt19937&r){if(!ok||(d-s).norm()>range)return false;std::bernoulli_distribution detection(0.96);return detection(r);} }

