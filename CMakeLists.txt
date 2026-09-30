cmake_minimum_required(VERSION 3.24)
project(RescueSwarm3D VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(RESCUESWARM_BUILD_RENDERER "Build the raylib + Dear ImGui visualizer" ON)
option(RESCUESWARM_BUILD_TESTS "Build tests" ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.11 REQUIRED)

add_library(rescueswarm_core
    src/planning/voxel_grid.cpp
    src/planning/astar_3d.cpp
    src/simulation/world.cpp
    src/simulation/drone.cpp
    src/simulation/simulation_clock.cpp
    src/autonomy/state_machine.cpp
    src/autonomy/frontier_explorer.cpp
    src/autonomy/task_allocator.cpp
    src/autonomy/collision_avoidance.cpp
    src/sensors/lidar.cpp
    src/sensors/gps.cpp
    src/sensors/survivor_detector.cpp
    src/telemetry/metrics.cpp)
target_include_directories(rescueswarm_core PUBLIC include)
target_link_libraries(rescueswarm_core PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json)
target_compile_options(rescueswarm_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(rescueswarm src/app/main.cpp)

if(RESCUESWARM_BUILD_RENDERER)
    find_package(raylib 5.0 REQUIRED)
    set(IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui")
    add_library(rescueswarm_ui
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        third_party/rlImGui/rlImGui.cpp
        src/rendering/renderer.cpp)
    target_include_directories(rescueswarm_ui PUBLIC include ${IMGUI_DIR} third_party/rlImGui)
    target_link_libraries(rescueswarm_ui PUBLIC rescueswarm_core raylib)
    target_compile_definitions(rescueswarm PRIVATE RESCUESWARM_WITH_RENDERER=1)
    target_link_libraries(rescueswarm PRIVATE rescueswarm_ui)
else()
    target_link_libraries(rescueswarm PRIVATE rescueswarm_core)
endif()

if(RESCUESWARM_BUILD_TESTS)
    include(CTest)
    find_package(Catch2 3 REQUIRED)
    add_executable(rescueswarm_tests
        tests/planner_tests.cpp
        tests/state_machine_tests.cpp
        tests/scenario_tests.cpp)
    target_link_libraries(rescueswarm_tests PRIVATE rescueswarm_core Catch2::Catch2WithMain)
    target_compile_definitions(rescueswarm_tests PRIVATE RESCUESWARM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    include(Catch)
    catch_discover_tests(rescueswarm_tests)
endif()
