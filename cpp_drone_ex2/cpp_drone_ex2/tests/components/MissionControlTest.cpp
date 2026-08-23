#include <gtest/gtest.h>
#include <drone_mapper/MissionControlImpl.h>
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
class MissionControl : public ::testing::Test {
protected:
    MissionConfigData mission_config = makeTestMissionConfig();
    DroneConfigData drone_config = makeTestDroneConfig();
    std::filesystem::path output_map_file = "./test_output.npy";
    NiceMock<tests::MockMap3D> hidden_map;
    NiceMock<tests::MockMutableMap3D> output_map;
    NiceMock<tests::MockDroneControl> drone_control;

    void SetUp() override {
        DroneState state;
        state.position = Position3D{0*x_extent[cm], 0*y_extent[cm], 0*z_extent[cm]};
        ON_CALL(drone_control, state()).WillByDefault(Return(state));
        
        MapConfig cfg = makeTestMapConfig();
        ON_CALL(output_map, getMapConfig()).WillByDefault(Return(cfg));
        ON_CALL(hidden_map, getMapConfig()).WillByDefault(Return(cfg));
        ON_CALL(output_map, isInBounds(_)).WillByDefault(Return(true));
        ON_CALL(hidden_map, isInBounds(_)).WillByDefault(Return(true));
        ON_CALL(hidden_map, atVoxel(_)).WillByDefault(Return(VoxelOccupancy::Unmapped));
    }
};

TEST_F(MissionControl, HappyPath_AlgorithmFinishes) {
    DroneStepResult res_continue{DroneStepStatus::Continue, ""};
    DroneStepResult res_complete{DroneStepStatus::Completed, ""};
    
    EXPECT_CALL(drone_control, step())
        .WillOnce(Return(res_continue))
        .WillOnce(Return(res_continue))
        .WillOnce(Return(res_complete));

    EXPECT_CALL(output_map, save(output_map_file)).Times(1);

    MissionControlImpl mc(mission_config, drone_config, hidden_map, output_map, drone_control, output_map_file);
    auto result = mc.runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 3);
    EXPECT_TRUE(result.errors.empty());
}

TEST_F(MissionControl, MaxStepsReached) {
    mission_config.max_steps = 5;
    DroneStepResult res_continue{DroneStepStatus::Continue, ""};
    
    EXPECT_CALL(drone_control, step())
        .Times(5)
        .WillRepeatedly(Return(res_continue));

    EXPECT_CALL(output_map, save(output_map_file)).Times(1);

    MissionControlImpl mc(mission_config, drone_config, hidden_map, output_map, drone_control, output_map_file);
    auto result = mc.runMission();

    EXPECT_EQ(result.status, MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 5);
}

TEST_F(MissionControl, DroneStepError_StopsImmediately) {
    DroneStepResult res_err{DroneStepStatus::Error, "Something failed"};
    
    EXPECT_CALL(drone_control, step()).WillOnce(Return(res_err));
    EXPECT_CALL(output_map, save(output_map_file)).Times(1);

    MissionControlImpl mc(mission_config, drone_config, hidden_map, output_map, drone_control, output_map_file);
    auto result = mc.runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 1);
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors[0].code, "DRONE_STEP_ERROR");
}

TEST_F(MissionControl, CollisionDetected_StopsWithError) {
    DroneStepResult res_continue{DroneStepStatus::Continue, ""};
    EXPECT_CALL(drone_control, step()).WillOnce(Return(res_continue));
    
    EXPECT_CALL(hidden_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));

    MissionControlImpl mc(mission_config, drone_config, hidden_map, output_map, drone_control, output_map_file);
    auto result = mc.runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Error);
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors[0].code, "ILLEGAL_MOVEMENT_COLLISION");
}

TEST_F(MissionControl, OutOfBoundsDetected_StopsWithError) {
    DroneState bad_state;
    bad_state.position = Position3D{100.0*x_extent[cm], 100.0*y_extent[cm], 100.0*z_extent[cm]};
    EXPECT_CALL(drone_control, state()).WillRepeatedly(Return(bad_state));
    DroneStepResult res_continue{DroneStepStatus::Continue, ""};
    EXPECT_CALL(drone_control, step()).WillOnce(Return(res_continue));

    EXPECT_CALL(output_map, save(output_map_file)).Times(1);

    MissionControlImpl mc(mission_config, drone_config, hidden_map, output_map, drone_control, output_map_file);
    auto result = mc.runMission();

    EXPECT_EQ(result.status, MissionRunStatus::Error);
    EXPECT_FALSE(result.errors.empty());
    if(!result.errors.empty()) {
        EXPECT_EQ(result.errors[0].code, "ILLEGAL_MOVEMENT_OUT_OF_BOUNDS");
    }
}

TEST_F(MissionControl, OutputMapSaved) {
    DroneStepResult res_complete{DroneStepStatus::Completed, ""};
    EXPECT_CALL(drone_control, step()).WillOnce(Return(res_complete));
    EXPECT_CALL(output_map, save(output_map_file)).Times(1);

    MissionControlImpl mc(mission_config, drone_config, hidden_map, output_map, drone_control, output_map_file);
    [[maybe_unused]] auto r1 = mc.runMission();
}

} // namespace
