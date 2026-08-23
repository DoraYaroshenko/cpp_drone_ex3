cmake_minimum_required(VERSION 3.16)
project(drone_mapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)


find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED) # new!

add_library(drone_mapper
    src/CollisionUtils.cpp
    src/DroneControlImpl.cpp
    src/MappingAlgorithmImpl.cpp
    src/MapsComparison.cpp
    src/MissionControlImpl.cpp
    src/Map3DImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/ScanResultToVoxels.cpp
    src/SimulationManager.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
    src/Logger.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
target_compile_options(drone_mapper
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp
)

target_precompile_headers(drone_mapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/drone_mapper/Units.h>
    <yaml-cpp/yaml.h>
)


add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)
#Compile the maps_comparison util
add_executable(maps_comparison src/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

# Tests
enable_testing()
find_package(GTest CONFIG REQUIRED)

file(GLOB_RECURSE TEST_SOURCES "tests/components/*.cpp" "tests/integration/*.cpp")


add_executable(drone_mapper_simulation_test ${TEST_SOURCES})
target_link_libraries(drone_mapper_simulation_test PRIVATE drone_mapper GTest::gtest_main GTest::gmock)

include(GoogleTest)
gtest_discover_tests(drone_mapper_simulation_test)
