#include "rescueswarm/simulation/world.hpp"
#include "rescueswarm/autonomy/autonomy.hpp"
#include "rescueswarm/planning/astar_3d.hpp"
#include "rescueswarm/sensors/sensors.hpp"
#include <algorithm>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace rs {
namespace {
Vec3 vec3(const nlohmann::json& j) { return {j.at(0).get<float>(), j.at(1).get<float>(), j.at(2).get<float>()}; }
bool sameCell(const Cell& a, const Cell& b) { return (a.array() == b.array()).all(); }
}

Scenario Scenario::load(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Could not open scenario: " + path);
    nlohmann::json j; input >> j; Scenario s;
    s.name = j.value("name", s.name);
    if (j.contains("dimensions")) s.dimensions = vec3(j["dimensions"]).cast<int>();
    s.resolution = j.value("resolution", s.resolution);
    s.drone_count = j.value("drone_count", s.drone_count);
    s.time_limit = j.value("time_limit", s.time_limit);
    if (j.contains("base")) s.base = vec3(j["base"]);
    for (const auto& o : j.value("obstacles", nlohmann::json::array()))
        s.obstacles.emplace_back(vec3(o.at("min")), vec3(o.at("max")));
    for (const auto& p : j.value("survivors", nlohmann::json::array())) s.survivors.push_back(vec3(p));
    if (j.contains("failures")) { const auto& f=j["failures"]; s.failures.gps_loss_probability=f.value("gps_loss_probability",s.failures.gps_loss_probability); s.failures.sensor_dropout_probability=f.value("sensor_dropout_probability",s.failures.sensor_dropout_probability); s.failures.comms_dropout_probability=f.value("comms_dropout_probability",s.failures.comms_dropout_probability); }
    return s;
}

World::World(Scenario scenario, unsigned seed)
    : scenario_(std::move(scenario)), truth_(scenario_.dimensions, scenario_.resolution), known_(scenario_.dimensions, scenario_.resolution), rng_(seed) {
    auto dims=truth_.dimensions();
    for(int y=0;y<dims.y();++y) for(int z=0;z<dims.z();++z) for(int x=0;x<dims.x();++x) {
        Cell c{x,y,z}; Vec3 p=truth_.cellCenter(c); bool occupied=(y==0||x==0||z==0||x==dims.x()-1||z==dims.z()-1);
        for(const auto& box:scenario_.obstacles) if(box.contains(p)){occupied=true;break;}
        truth_.set(c,occupied?Occupancy::Occupied:Occupancy::Free);
    }
    for(const auto& p:scenario_.survivors) survivors_.push_back({p,false,false});
    for(int i=0;i<scenario_.drone_count;++i){Drone d;d.id=i;d.position=scenario_.base+Vec3{0.7f*(i%2),0.25f,0.7f*(i/2)};d.state=DroneState::Idle;d.trajectory.push_back(d.position);drones_.push_back(d);}
    allocateRegions(drones_,std::max(1,scenario_.drone_count));
    for(auto&d:drones_) lidarScan(d.position,d.sensor_range,truth_,known_);
}

void World::plan(Drone& d,const Cell& goal){auto start=known_.worldToCell(d.position);auto result=astar3D(known_,start,goal,true);d.path.clear();d.mission_target=known_.cellCenter(goal);d.target_active=result.found;if(result.found)for(std::size_t i=1;i<result.cells.size();++i)d.path.push_back(known_.cellCenter(result.cells[i]));}

void World::injectFailures(Drone& d,float dt){
    std::uniform_real_distribution<float> u(0,1); auto toggle=[&](bool& available,float fail,float recover){if(available&&u(rng_)<fail*dt)available=false;else if(!available&&u(rng_)<recover*dt)available=true;};
    bool gps=d.gps_available;toggle(d.gps_available,scenario_.failures.gps_loss_probability,0.22f);toggle(d.sensor_available,scenario_.failures.sensor_dropout_probability,0.35f);toggle(d.comms_available,scenario_.failures.comms_dropout_probability,0.30f);if(gps&&!d.gps_available)++gps_losses_;if(!gps&&d.gps_available)++gps_recoveries_;
}

void World::sense(Drone& d,float){if(d.sensor_available)lidarScan(d.position,d.sensor_range,truth_,known_);for(auto&s:survivors_)if(!s.confirmed&&survivorDetected(d.position,s.position,d.sensor_range,d.sensor_available,rng_)){s.detected=true;if((d.position-s.position).norm()<1.8f){s.confirmed=true;++d.detections;}}}

void World::assignSurvivorTasks(){
    std::vector<bool> used(drones_.size(),false);
    for(auto&d:drones_)d.assigned_survivor=-1;
    for(std::size_t si=0;si<survivors_.size();++si){const auto&s=survivors_[si];if(!s.detected||s.confirmed)continue;int best=-1;float best_cost=std::numeric_limits<float>::infinity();for(std::size_t di=0;di<drones_.size();++di){const auto&d=drones_[di];if(used[di]||d.state==DroneState::Failed||d.state==DroneState::ReturnToBase||d.state==DroneState::Land)continue;float cost=(d.position-s.position).norm()+0.35f*(100.0f-d.battery);if(!d.comms_available&&cost>d.sensor_range)cost+=1000.0f;if(cost<best_cost){best_cost=cost;best=static_cast<int>(di);}}if(best>=0){drones_[best].assigned_survivor=static_cast<int>(si);used[best]=true;}}
}

void World::updateAutonomy(Drone& d,float dt){
    bool all_found=std::all_of(survivors_.begin(),survivors_.end(),[](const Survivor&s){return s.confirmed;});
    const bool at_base=(d.position-scenario_.base).norm()<1.2f;
    Survivor* assigned=nullptr;if(d.assigned_survivor>=0&&d.assigned_survivor<static_cast<int>(survivors_.size())&&!survivors_[d.assigned_survivor].confirmed)assigned=&survivors_[d.assigned_survivor];
    auto nearby=std::find_if(survivors_.begin(),survivors_.end(),[&](const Survivor&s){return s.detected&&!s.confirmed&&(d.position-s.position).norm()<d.sensor_range;});
    bool obstacle=false;
    if(!d.path.empty() && known_.at(known_.worldToCell(d.path.front()))==Occupancy::Occupied){d.path.clear();obstacle=true;}
    bool owns_visible_survivor=assigned&&nearby!=survivors_.end()&&(&*nearby==assigned);float return_reserve=(d.position-scenario_.base).norm()*0.18f;d.state=nextState(d.state,d.battery-return_reserve,!d.path.empty(),owns_visible_survivor,at_base,obstacle);
    if(assigned&&nearby==survivors_.end()&&d.state!=DroneState::ReturnToBase&&d.state!=DroneState::Land)d.state=DroneState::Investigate;
    if(!assigned&&(d.state==DroneState::Investigate||d.state==DroneState::ConfirmSurvivor))d.state=DroneState::Explore;
    if(all_found&&d.state!=DroneState::Land)d.state=at_base?DroneState::Land:DroneState::ReturnToBase;
    if(d.battery<=0){d.state=DroneState::Failed;d.velocity.setZero();return;}
    if(d.state==DroneState::Land){d.velocity*=0.8f;return;}
    if(d.state==DroneState::ConfirmSurvivor&&owns_visible_survivor&&(d.position-assigned->position).norm()<2.2f){assigned->confirmed=true;++d.detections;d.path.clear();d.target_active=false;d.state=DroneState::Explore;}
    if(d.path.empty()){
        if(d.state==DroneState::ReturnToBase) plan(d,known_.worldToCell(scenario_.base));
        else if(assigned) plan(d,known_.worldToCell(assigned->position));
        else if(nearby!=survivors_.end()) plan(d,known_.worldToCell(nearby->position));
        else {
            Cell here=known_.worldToCell(d.position);
            Cell frontier=chooseFrontierScored(known_,d,drones_);
            if(sameCell(frontier,here)){
                int regions=std::max(1,scenario_.drone_count);
                Cell staging{std::clamp((2*d.assigned_region+1)*known_.dimensions().x()/(2*regions),1,known_.dimensions().x()-2),2,known_.dimensions().z()/2};
                if((staging-here).cwiseAbs().sum()>static_cast<int>(d.sensor_range)) frontier=staging;
                else frontier=chooseFrontier(known_,here,-1,0);
            }
            plan(d,frontier);
        }
    }
    Vec3 desired=Vec3::Zero();if(!d.path.empty()){Vec3 delta=d.path.front()-d.position;if(delta.norm()<0.45f){d.path.pop_front();if(d.path.empty())d.target_active=false;}else desired=delta.normalized()*d.max_speed;}
    desired+=collisionAvoidance(d,drones_,truth_);if(desired.norm()>d.max_speed)desired=desired.normalized()*d.max_speed;
    Vec3 projected_dv=desired-d.velocity;
    if(projected_dv.norm()>d.max_acceleration)projected_dv=projected_dv.normalized()*d.max_acceleration;
    Vec3 projected_velocity=d.velocity+projected_dv*dt;
    if(projected_velocity.norm()>d.max_speed)projected_velocity=projected_velocity.normalized()*d.max_speed;
    Cell projected=known_.worldToCell(d.position+projected_velocity*dt);
    if(known_.at(projected)==Occupancy::Occupied){desired.setZero();d.velocity.setZero();d.path.clear();d.target_active=false;d.state=DroneState::Replan;++d.replans;}
    Vec3 old=d.position;d.integrate(dt,desired);
    Cell now=truth_.worldToCell(d.position);if(truth_.at(now)==Occupancy::Occupied){d.position=old;d.velocity=-0.2f*d.velocity;d.path.clear();d.target_active=false;d.state=DroneState::Replan;++d.replans;++collisions_;}
}

void World::tick(float dt){if(paused||finished())return;time_+=dt;for(auto&d:drones_){injectFailures(d,dt);sense(d,dt);}assignSurvivorTasks();for(auto&d:drones_)updateAutonomy(d,dt);}
bool World::finished()const{bool terminal=std::all_of(drones_.begin(),drones_.end(),[](const Drone&d){return d.state==DroneState::Land||d.state==DroneState::Failed;});return time_>=scenario_.time_limit||terminal;}
MissionMetrics World::metrics()const{MissionMetrics m;m.survivor_count=static_cast<int>(survivors_.size());m.survivors_found=static_cast<int>(std::count_if(survivors_.begin(),survivors_.end(),[](const Survivor&s){return s.confirmed;}));m.completed=m.survivors_found==m.survivor_count&&std::none_of(drones_.begin(),drones_.end(),[](const Drone&d){return d.state==DroneState::Failed;});auto free=truth_.count(Occupancy::Free);m.coverage=free?static_cast<float>(known_.count(Occupancy::Free))/static_cast<float>(free):0;m.collisions=collisions_>0?1:0;m.gps_losses=gps_losses_;m.gps_recoveries=gps_recoveries_;m.completion_time=time_;return m;}
}
