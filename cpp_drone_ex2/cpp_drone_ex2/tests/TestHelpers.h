#pragma once

#include <drone_mapper/Types.h>
#include <TinyNPY.h>
#include <memory>
#include <vector>
#include <cstring>
#include <filesystem>

namespace drone_mapper::tests {

inline drone_mapper::types::MapConfig makeTestMapConfig() {
    drone_mapper::types::MapConfig config;
    config.resolution = 10.0 * cm;
    config.offset = Position3D{-50.0 * x_extent[cm], -50.0 * y_extent[cm], -50.0 * z_extent[cm]};
    config.boundaries.min_x = -50.0 * x_extent[cm];
    config.boundaries.max_x = 50.0 * x_extent[cm];
    config.boundaries.min_y = -50.0 * y_extent[cm];
    config.boundaries.max_y = 50.0 * y_extent[cm];
    config.boundaries.min_height = -50.0 * z_extent[cm];
    config.boundaries.max_height = 50.0 * z_extent[cm];
    return config;
}

inline drone_mapper::types::DroneConfigData makeTestDroneConfig() {
    drone_mapper::types::DroneConfigData config;
    config.radius = 15.0 * cm;
    config.max_rotate = 45.0 * deg;
    config.max_advance = 50.0 * cm;
    config.max_elevate = 40.0 * cm;
    return config;
}

inline drone_mapper::types::LidarConfigData makeTestLidarConfig() {
    drone_mapper::types::LidarConfigData config;
    config.z_min = 20.0 * cm;
    config.z_max = 120.0 * cm;
    config.d = 2.5 * cm;
    config.fov_circles = 5;
    return config;
}

inline drone_mapper::types::MissionConfigData makeTestMissionConfig() {
    drone_mapper::types::MissionConfigData config;
    config.max_steps = 1000;
    config.gps_resolution = 10.0 * cm;
    config.output_mapping_resolution_factor = 1.0;
    config.mission_bounds.min_x = -50.0 * x_extent[cm];
    config.mission_bounds.max_x = 50.0 * x_extent[cm];
    config.mission_bounds.min_y = -50.0 * y_extent[cm];
    config.mission_bounds.max_y = 50.0 * y_extent[cm];
    config.mission_bounds.min_height = -50.0 * z_extent[cm];
    config.mission_bounds.max_height = 50.0 * z_extent[cm];
    return config;
}

inline drone_mapper::types::SimulationConfigData makeTestSimulationConfig() {
    drone_mapper::types::SimulationConfigData config;
    config.map_filename = "test_map.npy";
    config.map_resolution = 10.0 * cm;
    config.initial_drone_position = Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    config.initial_angle = 0.0 * deg;
    config.map_offset = Position3D{-50.0 * x_extent[cm], -50.0 * y_extent[cm], -50.0 * z_extent[cm]};
    return config;
}
// Helper from SimulationRunFactoryImpl to create a .npy file array
inline std::shared_ptr<NpyArray> createEmptyNpyArray(std::size_t sx, std::size_t sy, std::size_t sz, int default_value) {
    std::vector<std::size_t> shape = {sx, sy, sz};
    std::size_t total = sx * sy * sz;
    std::vector<int> data(total, default_value);

    auto arr = std::make_shared<NpyArray>(shape, sizeof(int), NpyArray::GetTypeChar(typeid(int)));
    arr->Allocate();
    std::memcpy(arr->Data<int>(), data.data(), arr->SizeBytes());
    return arr;
}

inline void saveTestNpyArray(const std::filesystem::path& path, std::size_t sx, std::size_t sy, std::size_t sz, int default_value) {
    auto arr = createEmptyNpyArray(sx, sy, sz, default_value);
    arr->SaveNPY(path.string().c_str());
}

} // namespace drone_mapper::tests
