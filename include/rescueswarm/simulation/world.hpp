#pragma once
#include "rescueswarm/simulation/drone.hpp"
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <vector>
namespace rs {
struct Survivor { Vec3 position{0,0,0}; bool detected = false; bool confirmed = false; };
struct FailureConfig { float gps_loss_probability=0.002f, sensor_dropout_probability=0.001f, comms_dropout_probability=0.003f; };
struct Scenario {
    std::string name = "Urban Search";
    Cell dimensions{40, 12, 40};
    float resolution = 1.0f;
    int drone_count = 4;
    float time_limit = 240.0f;
    Vec3 base{2.5f, 1.5f, 2.5f};
    std::vector<Eigen::AlignedBox3f> obstacles;
    std::vector<Vec3> survivors;
    FailureConfig failures;
    static Scenario load(const std::string& path);
};

struct MissionMetrics {
    bool completed = false;
    float coverage = 0.0f;
    int survivors_found = 0;
    int survivor_count = 0;
    int collisions = 0;
    int gps_losses = 0;
    int gps_recoveries = 0;
    float completion_time = 0.0f;
};

class World {
public:
    World(Scenario scenario, unsigned seed);
    void tick(float dt);
    bool finished() const;
    MissionMetrics metrics() const;
    const Scenario& scenario() const { return scenario_; }
    const VoxelGrid& truth() const { return truth_; }
    const VoxelGrid& known() const { return known_; }
    const std::vector<Drone>& drones() const { return drones_; }
    const std::vector<Survivor>& survivors() const { return survivors_; }
    float time() const { return time_; }
    bool paused = false;
    float time_scale = 2.0f;
private:
    void sense(Drone& drone, float dt);
    void updateAutonomy(Drone& drone, float dt);
    void assignSurvivorTasks();
    void plan(Drone& drone, const Cell& goal);
    void injectFailures(Drone& drone, float dt);
    Scenario scenario_;
    VoxelGrid truth_, known_;
    std::vector<Drone> drones_;
    std::vector<Survivor> survivors_;
    std::mt19937 rng_;
    float time_ = 0.0f;
    int collisions_ = 0, gps_losses_ = 0, gps_recoveries_ = 0;
};
}
