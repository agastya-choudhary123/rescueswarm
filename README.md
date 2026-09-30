rescueswarm
-----------

rescueswarm is a multi-agent search-and-rescue autonomy simulator written in
C++20. Four drones map a damaged city, divide exploration work, detect and
confirm survivors, replan around newly observed obstacles, budget energy for
the return flight, and recover from intermittent GPS, sensor and communication
failures.

The renderer and simulator are separate targets. The same mission can run as an
interactive raylib + Dear ImGui visualization or headlessly at roughly 684x
simulated time. In a 500-seed city benchmark the swarm completed 500/500
missions, recovered 100% of survivors, recorded no collision missions and
recovered from 95.6% of GPS-loss events.

![RescueSwarm live mission](render-final-stable.png)

### Documentation quick links

* [Quick start](#quick-start)
* [Autonomy](#autonomy)
* [Simulation](#simulation)
* [Visualization](#visualization)
* [Results](#results)
* [Failure injection](#failure-injection)
* [Limitations](#limitations)
* [Tests](#tests)

### Requirements

macOS or Linux with a C++20 compiler and CMake 3.24+. The interactive build
uses raylib 6, Eigen, nlohmann/json and Catch2. Dear ImGui and rlImGui are
vendored in `third_party/`.

On macOS:

```
$ brew install cmake raylib eigen catch2 nlohmann-json
```

### Quick start

On macOS, double-click `Launch RescueSwarm.command`. It builds the project if
needed and starts the city mission.

From a terminal:

```
$ cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
$ cmake --build build -j
$ ./build/rescueswarm --scenario scenarios/city.json --seed 42
```

The mission starts at 2x simulated time. The control panel can pause it or move
between 0.25x and 8x. Hold right mouse and use WASD to move the camera; camera
input is gated behind the button so an unattended view never drifts.

Headless runs use the identical `World`, planner, sensors and state machines:

```
$ ./build/rescueswarm --scenario scenarios/city.json --seed 42 --headless
$ ./build/rescueswarm --scenario scenarios/city.json --benchmark 500 --headless
```

### Autonomy

Each drone owns kinematic state, a local mission state, a planned path, a
battery budget, sensor and communication health, a region assignment and an
optional survivor task.

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

The exploration policy is not a random walk. LiDAR observations update a 3D
occupancy grid, and a frontier is a known-free cell on the operational flight
layer adjacent to unknown space. Candidates are scored by:

* expected information gain in a five-by-five neighborhood;
* A*-distance from the drone;
* whether the cell belongs to the drone's allocated city sector; and
* separation from teammates' current targets.

Restricting frontier generation to the operational layer matters. An earlier
version treated every unknown air voxel as useful and spent most of its search
budget mapping empty space above buildings. The 3D grid and planner remain
fully volumetric; only the exploration objective encodes where survivors are
expected to exist.

When any drone detects a survivor, a centralized allocator selects one eligible
drone by travel distance, remaining battery and communication state. Assignment
is exclusive, so nearby teammates continue their sectors instead of duplicating
the confirmation flight. The selected drone switches to `Investigate`, follows
a 3D A* path, confirms within 2.2 m, then rejoins exploration.

Paths are invalidated as soon as sensing marks the next voxel occupied. The
drone brakes before crossing the cell boundary, replans against the updated
map, and adds short-range separation forces for teammates and structures.
Return decisions include a distance-derived energy reserve rather than waiting
for a fixed battery threshold regardless of location.

### Simulation

`SimulationClock` accumulates render-frame time and advances the world at a
fixed 30 Hz. Rendering FPS therefore changes visual smoothness, not trajectories
or mission results. A fixed seed reproduces sensor detections, failures,
assignments and final metrics exactly.

Each drone tracks 3D position, velocity, acceleration and orientation with
bounded acceleration and speed. This is deliberately a kinematic model rather
than a rotor/aerodynamics model: the engineering target is planning,
coordination and failure recovery, not flight-controller tuning.

Scenarios are JSON and define grid dimensions, resolution, base location,
building volumes, survivors, drone count, time limit and per-second failure
probabilities. `city.json` is a 40 x 12 x 40 voxel environment with ten damaged
structures, four drones and four survivors. `training.json` is a smaller,
failure-free correctness scenario.

The simulation owns no raylib or ImGui types. `rescueswarm_core` can be linked
into benchmarks and tests without creating a window.

### Visualization

The renderer exposes the autonomy rather than hiding it behind realistic
models:

* color-coded quadrotors with state labels, trajectories and A* waypoints;
* mission targets, survivor pulses and detection beacons;
* LiDAR range rings and live inter-drone communication links;
* procedurally generated buildings, windows, roads and rubble;
* optional occupied-voxel overlays; and
* live battery, task ownership, replans, sensor health, coverage and mission
  progress in Dear ImGui.

The default camera frames the entire city and remains fixed until right-mouse
camera control is engaged. Labels are projected from world coordinates after
the 3D pass, so drones remain identifiable when geometry partially occludes
their bodies.

### Results

Apple M4, release build, 500 consecutive seeds of `scenarios/city.json`:

```
$ /usr/bin/time -p ./build/rescueswarm \
      --scenario scenarios/city.json --benchmark 500 --headless

Missions completed:       500/500
Mean area coverage:       83.8%
Survivors found:          100.0%
Collision rate:           0.0%
GPS-loss recovery rate:   95.6%
Mean completion time:     103.0 s
real 75.27
```

That is 6.6 complete missions per wall-clock second. The runs represent 51,500
seconds of simulated mission time, so the headless engine advances about 684x
real time on this machine.

`Missions completed` requires every survivor confirmed and no failed drone.
`Collision rate` is the percentage of missions containing at least one
collision, not collisions divided by missions. `Area coverage` is known free
volume divided by true free volume; it no longer approaches 100% by sending
drones into irrelevant high-altitude air.

The benchmark is deterministic by seed but not hard-coded: every run advances
the same 30 Hz simulation used by the GUI until the team lands or the time limit
expires. Metrics are aggregated from resulting world state.

### Failure injection

The city scenario independently injects GPS loss, sensor dropout and
communication dropout on simulation ticks. Failed subsystems recover
probabilistically, so outage duration varies across seeds.

GPS health and recovery are tracked explicitly. A sensor outage stops occupancy
and survivor observations; existing paths continue until sensing returns or
another known obstacle invalidates them. A communications outage makes a
disconnected drone expensive to assign to a remotely reported survivor, while
allowing it to continue its current sector autonomously.

### Limitations

The map is currently shared in-process. Communication state affects task
allocation and telemetry, but it does not yet maintain divergent per-drone maps
or reconcile them after a partition. A truly distributed allocation experiment
needs one occupancy map and frontier set per agent.

Survivor sensing is range-based with probabilistic detection, not ray-occluded;
a building can attenuate neither LiDAR nor the survivor detector. LiDAR reveals
voxels in a sphere rather than casting individual rays.

A* replans from scratch. D* Lite remains the natural next planner because most
updates change only a small part of the voxel graph.

The flight model is kinematic, buildings are axis-aligned volumes, and the
procedural damage is visual only. There is no rotor dynamics, wind, debris
physics or contact solver.

The current benchmark compares seeds, not planners. A* versus D* Lite and
centralized versus distributed allocation should only be reported after both
alternatives exist behind the same scenario harness.

### Tests

```
$ ctest --test-dir build --output-on-failure

100% tests passed, 0 tests failed out of 6
```

The suite checks 3D A* around occupied voxels, blocked-goal rejection,
battery-triggered return and landing, survivor-state transitions, operational
altitude frontiers, deterministic scenario loading and identical seeded world
evolution.

### Layout

```
include/rescueswarm/
  autonomy/       state transitions, frontier scoring, allocation, avoidance
  planning/       voxel occupancy grid and 3D A*
  sensors/        LiDAR, GPS and survivor detector interfaces
  simulation/     world, drones and fixed-step clock
  telemetry/      mission and aggregate metrics
  rendering/      renderer interface only
src/
  autonomy/       team policy and recovery behavior
  planning/       search and grid implementations
  sensors/        noisy sensor models
  simulation/     headless mission engine
  rendering/      raylib + Dear ImGui client
  app/            CLI, benchmark and interactive entry point
scenarios/        JSON mission definitions
tests/            Catch2 planner, state-machine and scenario tests
third_party/      Dear ImGui and rlImGui
```
