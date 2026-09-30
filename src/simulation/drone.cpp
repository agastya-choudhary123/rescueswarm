#include "rescueswarm/simulation/drone.hpp"
#include <algorithm>
namespace rs {
const char* stateName(DroneState s){switch(s){case DroneState::Idle:return"Idle";case DroneState::Explore:return"Explore";case DroneState::Investigate:return"Investigate";case DroneState::ConfirmSurvivor:return"Confirm Survivor";case DroneState::AvoidObstacle:return"Avoid Obstacle";case DroneState::Replan:return"Replan";case DroneState::ReturnToBase:return"Return to Base";case DroneState::Land:return"Land";default:return"Failed";}}
void Drone::integrate(float dt,const Vec3& desired){Vec3 dv=desired-velocity; if(dv.norm()>max_acceleration)dv=dv.normalized()*max_acceleration;acceleration=dv;velocity+=acceleration*dt;if(velocity.norm()>max_speed)velocity=velocity.normalized()*max_speed;position+=velocity*dt;if(velocity.squaredNorm()>0.01f)orientation.y()=std::atan2(velocity.x(),velocity.z());battery=std::max(0.0f,battery-dt*(0.035f+0.012f*velocity.squaredNorm()));state_time+=dt;if(trajectory.empty()||(trajectory.back()-position).norm()>0.35f)trajectory.push_back(position);}
}

