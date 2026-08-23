#pragma once

#include <gmock/gmock.h>

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMissionControl.h>
#include <drone_mapper/IMutableMap3D.h>
#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/Types.h>

namespace drone_mapper::tests {

class MockDroneControl : public IDroneControl {
public:
    MOCK_METHOD(types::DroneStepResult, step, (), (override));
    MOCK_METHOD(types::DroneState, state, (), (const, override));
};

class MockDroneMovement : public IDroneMovement {
public:
    MOCK_METHOD(types::MovementResult, rotate, (types::RotationDirection direction, HorizontalAngle angle), (override));
    MOCK_METHOD(types::MovementResult, advance, (PhysicalLength distance), (override));
    MOCK_METHOD(types::MovementResult, elevate, (PhysicalLength distance), (override));
};

class MockGPS : public IGPS {
public:
    MOCK_METHOD(Position3D, position, (), (const, override));
    MOCK_METHOD(Orientation, heading, (), (const, override));
};

class MockLidar : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation scan_orientation), (const, override));
    MOCK_METHOD(types::LidarConfigData, config, (), (const, override));
};

class MockMap3D : public IMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel, (const Position3D& pos), (const, override));
    MOCK_METHOD(types::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const Position3D& pos), (const, override));
};

class MockMutableMap3D : public IMutableMap3D {
public:
    MOCK_METHOD(types::VoxelOccupancy, atVoxel, (const Position3D& pos), (const, override));
    MOCK_METHOD(types::MapConfig, getMapConfig, (), (const, override));
    MOCK_METHOD(bool, isInBounds, (const Position3D& pos), (const, override));
    MOCK_METHOD(void, set, (const Position3D& pos, types::VoxelOccupancy value), (override));
    MOCK_METHOD(void, save, (const std::filesystem::path& path), (const, override));
};

class MockMappingAlgorithm : public IMappingAlgorithm {
public:
    MockMappingAlgorithm(const types::MissionConfigData& mission_config,
                         const types::LidarConfigData& lidar_config,
                         const types::DroneConfigData& drone_config,
                         const IMap3D& output_map)
        : IMappingAlgorithm(mission_config, lidar_config, drone_config, output_map) {}

    MOCK_METHOD(types::MappingStepCommand, nextStep, (const types::DroneState& state, const types::LidarScanResult* latest_scan), (override));
};

class MockMissionControl : public IMissionControl {
public:
    MOCK_METHOD(types::MissionRunResult, runMission, (), (override));
};

class MockSimulationRun : public ISimulationRun {
public:
    MOCK_METHOD(types::SimulationResult, run, (), (override));
};

class MockSimulationRunFactory : public ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<ISimulationRun>, create, (const types::SimulationConfigData& simulation, const types::MissionConfigData& mission, const types::DroneConfigData& drone, const types::LidarConfigData& lidar, const std::filesystem::path& output_path), (override));
};

} // namespace drone_mapper::tests
