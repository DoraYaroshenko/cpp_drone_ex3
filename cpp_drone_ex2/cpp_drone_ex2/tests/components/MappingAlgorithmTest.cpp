#include <gtest/gtest.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/Types.h>
#include "../mocks/Mocks.h"
#include "../TestHelpers.h"

using namespace drone_mapper;
using namespace drone_mapper::types;
using namespace drone_mapper::tests;
using ::testing::_;
using ::testing::Return;
using ::testing::NiceMock;

namespace {
class MappingAlgorithm : public ::testing::Test {
protected:
    NiceMock<tests::MockMap3D> output_map;
    MissionConfigData mission_config = makeTestMissionConfig();
    LidarConfigData lidar_config = makeTestLidarConfig();
    DroneConfigData drone_config = makeTestDroneConfig();
    DroneState state;

    void SetUp() override {
        state.position = Position3D{0*x_extent[cm], 0*y_extent[cm], 0*z_extent[cm]};
        state.heading = Orientation{0*deg, 0*deg};
        state.step_index = 0;
        
        MapConfig cfg = makeTestMapConfig();
        ON_CALL(output_map, getMapConfig()).WillByDefault(Return(cfg));
        ON_CALL(output_map, isInBounds(_)).WillByDefault(Return(true));
        ON_CALL(output_map, atVoxel(_)).WillByDefault(Return(VoxelOccupancy::Unmapped));
    }
};

TEST_F(MappingAlgorithm, NextStep_ReturnsValidMovementCommand) {
    MappingAlgorithmImpl algo(mission_config, lidar_config, drone_config, output_map);
    LidarScanResult scan; 
    
    auto cmd = algo.nextStep(state, &scan);
    
    EXPECT_TRUE(cmd.status == AlgorithmStatus::Working || cmd.status == AlgorithmStatus::Finished);
    if (cmd.status == AlgorithmStatus::Working) {
        EXPECT_TRUE(cmd.movement.has_value() || cmd.scan_orientation.has_value());
    }
}

TEST_F(MappingAlgorithm, NextStep_RespectsDroneCapabilities) {
    MappingAlgorithmImpl algo(mission_config, lidar_config, drone_config, output_map);
    LidarScanResult scan;
    
    for (int i=0; i<100; ++i) { 
        state.step_index = i;
        auto cmd = algo.nextStep(state, &scan);
        if (cmd.status == AlgorithmStatus::Finished) break;
        if (cmd.movement.has_value()) {
            if (cmd.movement->type == MovementCommandType::Advance) {
                EXPECT_LE(cmd.movement->distance.numerical_value_in(cm), drone_config.max_advance.numerical_value_in(cm) + 1e-6);
            }
            else if (cmd.movement->type == MovementCommandType::Rotate) {
                EXPECT_LE(cmd.movement->angle.numerical_value_in(deg), drone_config.max_rotate.numerical_value_in(deg) + 1e-6);
            }
            else if (cmd.movement->type == MovementCommandType::Elevate) {
                EXPECT_LE(std::abs(cmd.movement->distance.numerical_value_in(cm)), drone_config.max_elevate.numerical_value_in(cm) + 1e-6);
            }
        }
    }
}

TEST_F(MappingAlgorithm, NextStep_ReturnsValidStatus) {
    MappingAlgorithmImpl algo(mission_config, lidar_config, drone_config, output_map);
    LidarScanResult scan;
    
    auto cmd = algo.nextStep(state, &scan);
    EXPECT_TRUE(cmd.status == AlgorithmStatus::Working || cmd.status == AlgorithmStatus::Finished || cmd.status == AlgorithmStatus::FinishedWithUnmappableVoxels);
}


} // namespace
