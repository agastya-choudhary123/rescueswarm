#pragma once
#include "rescueswarm/planning/voxel_grid.hpp"
#include <deque>
#include <string>
#include <vector>
namespace rs {
enum class DroneState { Idle, Explore, Investigate, ConfirmSurvivor, AvoidObstacle, Replan, ReturnToBase, Land, Failed };
const char* stateName(DroneState state);

struct Drone {
    int id = 0;
    Vec3 position{0, 1, 0};
    Vec3 velocity{0, 0, 0};
    Vec3 acceleration{0, 0, 0};
    Vec3 orientation{0, 0, 0};
    float max_speed = 6.0f;
    float max_acceleration = 8.0f;
    float battery = 100.0f;
    float radius = 0.35f;
    float sensor_range = 8.0f;
    float communication_radius = 14.0f;
    DroneState state = DroneState::Idle;
    std::deque<Vec3> path;
    std::vector<Vec3> trajectory;
    Vec3 mission_target{0, 0, 0};
    bool target_active = false;
    int assigned_region = -1;
    int assigned_survivor = -1;
    int replans = 0;
    int detections = 0;
    bool gps_available = true;
    bool comms_available = true;
    bool sensor_available = true;
    float state_time = 0.0f;
    void integrate(float dt, const Vec3& desired_velocity);
};
}
