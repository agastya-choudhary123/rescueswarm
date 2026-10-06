# rescueswarm

A multi-drone search-and-rescue simulator written in C++20. Four drones
explore a collapsed city block they have no map of. They split the area into
sectors, find and confirm survivors, replan around obstacles as they discover
them, and save enough battery to fly home. GPS, sensor, and radio failures are
injected at random throughout.

The simulation is a library on its own. It runs either with a raylib + Dear
ImGui viewer or headless, which is about 740x faster than real time. A given
seed always reproduces the same mission.

Over 500 seeded missions, all 500 finished, every survivor was found, no
mission had a collision, and drones recovered from 95.6% of GPS outages.

![rescueswarm mission](assets/screenshot.png)

## Building

You need macOS or Linux, a C++20 compiler, and CMake 3.24+. The dependencies
are raylib 6, Eigen, nlohmann/json, and Catch2. Dear ImGui and rlImGui are
included as submodules.

```sh
brew install cmake raylib eigen catch2 nlohmann-json   # macOS

git clone --recursive https://github.com/agastya-choudhary123/rescueswarm
cd rescueswarm
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/rescueswarm --scenario scenarios/city.json --seed 42
```

If you cloned without `--recursive`, run `git submodule update --init` first.
On macOS you can also double-click `Launch RescueSwarm.command`.

In the viewer, hold the right mouse button and use WASD to move the camera.
The control panel can pause the sim or change its speed from 0.25x to 8x.

Headless:

```sh
./build/rescueswarm --scenario scenarios/city.json --seed 42 --headless
./build/rescueswarm --scenario scenarios/city.json --benchmark 500 --headless
```

## Results

500 consecutive seeds of `scenarios/city.json`, release build, Apple M4:

| metric | value | definition |
|---|---:|---|
| missions completed | 500/500 | every survivor confirmed and no drone lost |
| survivors found | 100.0% | confirmed / placed |
| collision rate | 0.0% | missions with at least one collision |
| GPS-loss recovery | 95.6% | GPS outages recovered from before the mission ended |
| area coverage | 83.8% | known free volume / true free volume |
| mean completion | 103.0 s | simulated time until the team lands |

The whole run took 69.4 s of wall-clock time for 51,500 s of simulated
flight. Headless mode steps the same 30 Hz simulation the viewer uses, and
the metrics come from the final world state. The `training.json` scenario,
which has no failures, finishes in 39.5 s with 99.0% coverage.

## How the drones work

```
                         survivor assigned
                                │
Idle ──► Explore ───────────────► Investigate ──► Confirm Survivor
             │                         │                  │
             │ new obstacle            └──── replan ──────┘
             ▼
      Avoid Obstacle ──► Replan
             │
             └──────── low energy ──► Return to Base ──► Land
```

**Exploration.** LiDAR updates a 3D occupancy grid. A frontier is a known-free
cell at flight altitude that sits next to unknown space. Each frontier is
scored on expected information gain (over a 5x5 neighborhood), A* distance,
whether it's in the drone's own sector, and how far it is from where
teammates are heading. Frontiers are limited to flight altitude because an
earlier version spent most of its time mapping empty sky above the buildings.
The grid and the planner are still fully 3D.

**Task allocation.** When a survivor is detected, a central allocator assigns
exactly one drone, picked by travel distance, battery, and radio state. The
other drones keep working their own sectors. The assigned drone flies a 3D A*
path, confirms the survivor from within 2.2 m, and goes back to exploring.

**Replanning and energy.** As soon as sensing marks the next voxel on a path
as occupied, the drone brakes and replans. Short-range separation forces keep
drones away from each other and from buildings. The return-home decision is
based on the energy needed to get back to base, not a fixed battery
percentage.

## Simulation

The world advances in fixed 30 Hz steps (`SimulationClock`), so frame rate
affects how smooth the viewer looks but never changes the results. Drones are
kinematic, with bounded speed and acceleration. This project is about
planning and coordination, not flight control.

| scenario | grid | buildings | drones | survivors | time limit | failures |
|---|---|---:|---:|---:|---:|---|
| `city.json` | 40 x 12 x 40, 1 m voxels | 10 | 4 | 4 | 300 s | on |
| `training.json` | 20 x 8 x 20, 1 m voxels | 2 | 2 | 2 | 120 s | off |

Failure rates in the city scenario. Each one is rolled independently on
every tick, and failed systems come back at random:

| failure | probability / s | effect |
|---|---:|---|
| GPS loss | 0.008 | counted toward the recovery rate |
| sensor dropout | 0.004 | no new map or survivor observations; the drone keeps following its current path |
| comms dropout | 0.012 | the drone keeps working its sector alone and becomes a worse pick for new survivors |

`rescueswarm_core` doesn't depend on raylib or ImGui, so the tests and
benchmarks never open a window.

The viewer shows each drone's state, trajectory, and A* waypoints, along with
survivor markers, LiDAR range, radio links, an optional occupied-voxel
overlay, and an ImGui panel with battery, task ownership, replans, sensor
health, and coverage.

## Limitations

- All drones share one in-process map. Radio state affects task allocation,
  but drones don't keep separate maps and merge them later.
- Survivor detection is range plus a random roll, with no line-of-sight
  check. LiDAR reveals a sphere of voxels instead of casting rays.
- A* replans from scratch. D* Lite would be the next step.
- Buildings are axis-aligned boxes, and there's no wind, rotor dynamics, or
  debris physics.
- The benchmark compares seeds, not algorithms. I haven't run comparisons
  like A* vs. D* Lite because the alternatives don't exist yet.

## Tests

```sh
ctest --test-dir build --output-on-failure
```

| file | checks |
|---|---|
| `planner_tests.cpp` | A* routes around occupied voxels; unreachable goals are rejected |
| `state_machine_tests.cpp` | low battery forces return and landing; detections trigger investigation; frontiers stay at flight altitude |
| `scenario_tests.cpp` | scenarios load; two worlds with the same seed evolve identically |

## Layout

```
include/rescueswarm/   headers: autonomy, planning, sensors, simulation, telemetry, rendering
src/autonomy/          state machine, frontier scoring, task allocation, collision avoidance
src/planning/          voxel grid and 3D A*
src/sensors/           LiDAR, GPS, survivor detector
src/simulation/        world, drones, fixed-step clock
src/rendering/         raylib + Dear ImGui viewer
src/app/main.cpp       CLI entry point
scenarios/             mission definitions
tests/                 Catch2 tests
third_party/           Dear ImGui, rlImGui
```
