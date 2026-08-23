#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

#include <drone_mapper/CollisionUtils.h>

namespace drone_mapper {

namespace {

// Create a NpyArray filled with a default int value.
std::shared_ptr<NpyArray> createEmptyNpyArray(std::size_t sx, std::size_t sy, std::size_t sz,
                                                int default_value) {
    std::vector<std::size_t> shape = {sx, sy, sz};
    std::size_t total = sx * sy * sz;
    std::vector<int> data(total, default_value);

    auto arr = std::make_shared<NpyArray>(shape, sizeof(int), NpyArray::GetTypeChar(typeid(int)));
    arr->Allocate();
    std::memcpy(arr->Data<int>(), data.data(), arr->SizeBytes());

    return arr;
}

// Load an NPY file into a shared NpyArray.
std::shared_ptr<NpyArray> loadNpyFile(const std::filesystem::path& path) {
    auto arr = std::make_shared<NpyArray>();
    const char* err = arr->LoadNPY(path.string().c_str());
    if (err != nullptr) {
        throw std::runtime_error(std::string("Failed to load NPY file '")
                                 + path.string() + "': " + err);
    }
    if (arr->Shape().size() != 3) {
        throw std::runtime_error("Expected 3D array [X, Y, Z] in NPY file: " + path.string());
    }
    return arr;
}

} // namespace

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {

    // 1. Load the hidden (ground truth) map.
    auto hidden_npy = loadNpyFile(simulation.map_filename);
    const auto& shape = hidden_npy->Shape(); // [X, Y, Z]

    // Build MapConfig for the hidden map.
    types::MapConfig hidden_config;
    hidden_config.resolution = simulation.map_resolution;
    hidden_config.offset = simulation.map_offset;

    PhysicalLength res = simulation.map_resolution;
    double res_cm = res.numerical_value_in(cm);
    
    hidden_config.boundaries.min_x = simulation.map_offset.x;
    hidden_config.boundaries.max_x = simulation.map_offset.x + (static_cast<double>(shape[0]) * res_cm * x_extent[cm]);
    hidden_config.boundaries.min_y = simulation.map_offset.y;
    hidden_config.boundaries.max_y = simulation.map_offset.y + (static_cast<double>(shape[1]) * res_cm * y_extent[cm]);
    hidden_config.boundaries.min_height = simulation.map_offset.z;
    hidden_config.boundaries.max_height = simulation.map_offset.z + (static_cast<double>(shape[2]) * res_cm * z_extent[cm]);

    auto hidden_map = std::make_unique<Map3DImpl>(std::move(hidden_npy), hidden_config);

    // 2. Create the output map with same dimensions, initialized to Unmapped (-1).
    auto output_npy = createEmptyNpyArray(
        shape[0], shape[1], shape[2],
        static_cast<int>(types::VoxelOccupancy::Unmapped)
    );

    // Output map config: same resolution and offset as hidden, but boundaries come from mission.
    types::MapConfig output_config;
    output_config.resolution = simulation.map_resolution;
    output_config.offset = simulation.map_offset;
    output_config.boundaries = mission.mission_bounds; 

    auto output_map = std::make_unique<Map3DImpl>(std::move(output_npy), output_config);

    // Validate mission boundaries against the physical hidden map boundaries
    if (mission.mission_bounds.min_x < hidden_config.boundaries.min_x ||
        mission.mission_bounds.max_x > hidden_config.boundaries.max_x ||
        mission.mission_bounds.min_y < hidden_config.boundaries.min_y ||
        mission.mission_bounds.max_y > hidden_config.boundaries.max_y ||
        mission.mission_bounds.min_height < hidden_config.boundaries.min_height ||
        mission.mission_bounds.max_height > hidden_config.boundaries.max_height) {
        throw std::invalid_argument("MISSION_BOUNDARY_INVALID");
    }

    // Initialize the voxels that are physically outside the mission boundaries to -2 (OutOfBounds).
    double res_cm_out = simulation.map_resolution.numerical_value_in(cm);
    for (std::size_t x = 0; x < shape[0]; ++x) {
        for (std::size_t y = 0; y < shape[1]; ++y) {
            for (std::size_t z = 0; z < shape[2]; ++z) {
                Position3D pos{
                    simulation.map_offset.x + XLength((x + 0.5) * res_cm_out * x_extent[cm]),
                    simulation.map_offset.y + YLength((y + 0.5) * res_cm_out * y_extent[cm]),
                    simulation.map_offset.z + ZLength((z + 0.5) * res_cm_out * z_extent[cm])
                };
                
                // output_map is configured with mission boundaries. If it's outside those bounds, set to -2.
                if (!output_map->isInBounds(pos)) {
                    output_map->set(pos, types::VoxelOccupancy::OutOfBounds);
                }
            }
        }
    }

    // Validate initial drone position against hidden map bounds
    if (!CollisionUtils::isDroneFullyInBounds(*hidden_map, simulation.initial_drone_position, drone.radius)) {
        throw std::invalid_argument("initial_drone_position's bounding box is outside hidden map boundaries");
    }

    // Validate initial drone position against mission bounds
    if (!CollisionUtils::isDroneFullyInBounds(*output_map, simulation.initial_drone_position, drone.radius)) {
        throw std::invalid_argument("initial_drone_position's bounding box is outside mission_boundaries");
    }

    // Validate initial drone position against obstacles
    if (CollisionUtils::isDroneColliding(*hidden_map, simulation.initial_drone_position, drone.radius)) {
        throw std::invalid_argument("initial_drone_position intersects with an occupied voxel in hidden map");
    }

    // 3. Create GPS (MockGPS).
    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, AltitudeAngle(0.0 * altitude_angle[deg])},
        mission.gps_resolution
    );

    // 4. Create movement driver.
    auto movement = std::make_unique<MockMovement>(*gps);

    // 5. Create lidar.
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);

    // 6. Create mapping algorithm.
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(
        mission, lidar, drone, *output_map
    );

    // 7. Create drone control.
    auto drone_control = std::make_unique<DroneControlImpl>(
        drone, mission, *lidar_impl, *gps, *movement, *output_map, *mapping_algorithm
    );

    // 8. Create mission control.
    std::filesystem::create_directories(output_path);
    const std::filesystem::path output_map_file = output_path / "map_output.npy";
    auto mission_control = std::make_unique<MissionControlImpl>(
        mission, drone, *hidden_map, *output_map, *drone_control, output_map_file
    );

    // 9. Assemble the simulation run.
    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file
    );
}

} // namespace drone_mapper
