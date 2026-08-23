#include <gtest/gtest.h>
#include <drone_mapper/SimulationRunImpl.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/Types.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include "../TestHelpers.h"
#include "IntegrationFixture.h"

using namespace drone_mapper;
using namespace drone_mapper::types;
using namespace drone_mapper::tests;

class ScriptedAlgorithm : public IMappingAlgorithm {
public:
    int step_count = 0;
    bool force_collision = false;

    ScriptedAlgorithm(const MissionConfigData& mission, const LidarConfigData& lidar, const DroneConfigData& drone, const IMap3D& map)
        : IMappingAlgorithm(mission, lidar, drone, map) {}

    MappingStepCommand nextStep(const DroneState&, const LidarScanResult*) override {
        MappingStepCommand cmd;
        if (force_collision) {
            cmd.movement = MovementCommand{MovementCommandType::Advance, RotationDirection::Left, 0*deg, 500*cm};
            cmd.status = AlgorithmStatus::Working;
            return cmd;
        }

        if (step_count < 5) {
            cmd.movement = MovementCommand{MovementCommandType::Advance, RotationDirection::Left, 0*deg, 5*cm};
            cmd.status = AlgorithmStatus::Working;
        } else {
            cmd.status = AlgorithmStatus::Finished;
        }
        step_count++;
        return cmd;
    }
};

class ScriptedSimulationRunFactory : public ISimulationRunFactory {
public:
    bool force_collision = false;

    std::unique_ptr<ISimulationRun> create(const SimulationConfigData& simulation,
                                           const MissionConfigData& mission,
                                           const DroneConfigData& drone,
                                           const LidarConfigData& lidar,
                                           const std::filesystem::path& output_path) override {
        
        auto hidden_npy = createEmptyNpyArray(10, 10, 10, 0);
        MapConfig cfg;
        cfg.resolution = simulation.map_resolution;
        cfg.offset = simulation.map_offset;
        cfg.boundaries = mission.mission_bounds;

        auto hidden_map = std::make_unique<Map3DImpl>(std::move(hidden_npy), cfg);

        auto output_npy = createEmptyNpyArray(10, 10, 10, 2);
        MapConfig out_cfg = cfg;
        out_cfg.boundaries = mission.mission_bounds;
        auto output_map = std::make_unique<Map3DImpl>(std::move(output_npy), out_cfg);

        auto gps = std::make_unique<MockGPS>(simulation.initial_drone_position, Orientation{simulation.initial_angle, 0*altitude_angle[deg]}, mission.gps_resolution);
        auto movement = std::make_unique<MockMovement>(*gps);
        auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);

        auto mapping_algorithm = std::make_unique<ScriptedAlgorithm>(mission, lidar, drone, *output_map);
        mapping_algorithm->force_collision = force_collision;

        auto drone_control = std::make_unique<DroneControlImpl>(drone, mission, *lidar_impl, *gps, *movement, *output_map, *mapping_algorithm);
        
        const std::filesystem::path output_map_file = output_path / "map_output.npy";
        auto mission_control = std::make_unique<MissionControlImpl>(mission, drone, *hidden_map, *output_map, *drone_control, output_map_file);

        return std::make_unique<SimulationRunImpl>(
            std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement), std::move(lidar_impl),
            std::move(mapping_algorithm), std::move(drone_control), std::move(mission_control),
            simulation, mission, output_map_file
        );
    }
};



TEST_F(Integration, MockAlgorithm_DeterministicPath_KnownScore) {
    auto factory = std::make_unique<ScriptedSimulationRunFactory>();
    factory->force_collision = false;
    SimulationManager manager(std::move(factory));

    auto report = manager.run(comp, output_path);
    ASSERT_EQ(report.runs.size(), 1);
    EXPECT_GE(report.runs[0].mission_score, 0.0);
}

TEST_F(Integration, MockAlgorithm_DroneCollision_DetectedAndRecorded) {
    auto factory = std::make_unique<ScriptedSimulationRunFactory>();
    factory->force_collision = true;
    SimulationManager manager(std::move(factory));

    auto report = manager.run(comp, output_path);
    ASSERT_EQ(report.runs.size(), 1);
    EXPECT_EQ(report.runs[0].mission_score, -1.0);
    EXPECT_EQ(report.runs[0].mission_results[0].status, MissionRunStatus::Error);
}
