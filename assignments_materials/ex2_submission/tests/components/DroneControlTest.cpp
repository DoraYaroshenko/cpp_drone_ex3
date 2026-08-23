#include <gtest/gtest.h>
#include <drone_mapper/DroneControlImpl.h>
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
class DroneControl : public ::testing::Test {
protected:
    DroneConfigData drone_config;
    MissionConfigData mission_config;
    NiceMock<tests::MockLidar> lidar;
    NiceMock<tests::MockGPS> gps;
    NiceMock<tests::MockDroneMovement> movement;
    NiceMock<tests::MockMutableMap3D> output_map;
    NiceMock<tests::MockMappingAlgorithm> mapping_algorithm{makeTestMissionConfig(), makeTestLidarConfig(), makeTestDroneConfig(), output_map};

    void SetUp() override {
        drone_config = makeTestDroneConfig();
        mission_config = makeTestMissionConfig();
        ON_CALL(gps, position()).WillByDefault(Return(Position3D{0*x_extent[cm], 0*y_extent[cm], 0*z_extent[cm]}));
        ON_CALL(gps, heading()).WillByDefault(Return(Orientation{0*deg, 0*deg}));
        ON_CALL(movement, advance(_)).WillByDefault(Return(MovementResult{true, ""}));
        ON_CALL(movement, rotate(_, _)).WillByDefault(Return(MovementResult{true, ""}));
        ON_CALL(movement, elevate(_)).WillByDefault(Return(MovementResult{true, ""}));
    }
};

TEST_F(DroneControl, HappyPath_SingleMovement) {
    MappingStepCommand cmd;
    cmd.status = AlgorithmStatus::Working;
    cmd.movement = MovementCommand{MovementCommandType::Advance, RotationDirection::Left, 0*deg, 10*cm};
    
    EXPECT_CALL(mapping_algorithm, nextStep(_, _)).WillOnce(Return(cmd));
    EXPECT_CALL(movement, advance(PhysicalLength(10*cm))).WillOnce(Return(MovementResult{true, ""}));

    DroneControlImpl drone(drone_config, mission_config, lidar, gps, movement, output_map, mapping_algorithm);
    auto res = drone.step();
    
    EXPECT_EQ(res.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, Chunking_AdvanceBeyondMax) {
    drone_config.max_advance = 10 * cm;
    
    MappingStepCommand cmd;
    cmd.status = AlgorithmStatus::Working;
    cmd.movement = MovementCommand{MovementCommandType::Advance, RotationDirection::Left, 0*deg, 25*cm};
    
    EXPECT_CALL(mapping_algorithm, nextStep(_, _)).WillOnce(Return(cmd));
    
    EXPECT_CALL(movement, advance(PhysicalLength(10*cm))).WillOnce(Return(MovementResult{true, ""}));

    DroneControlImpl drone(drone_config, mission_config, lidar, gps, movement, output_map, mapping_algorithm);
    auto res = drone.step();
    EXPECT_EQ(res.status, DroneStepStatus::Continue);
    
    EXPECT_CALL(movement, advance(PhysicalLength(10*cm))).WillOnce(Return(MovementResult{true, ""}));
    res = drone.step();
    EXPECT_EQ(res.status, DroneStepStatus::Continue);

    EXPECT_CALL(movement, advance(PhysicalLength(5*cm))).WillOnce(Return(MovementResult{true, ""}));
    res = drone.step();
    EXPECT_EQ(res.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, AlgorithmFinished_ReturnsCompleted) {
    MappingStepCommand cmd;
    cmd.status = AlgorithmStatus::Finished;
    
    EXPECT_CALL(mapping_algorithm, nextStep(_, _)).WillOnce(Return(cmd));

    DroneControlImpl drone(drone_config, mission_config, lidar, gps, movement, output_map, mapping_algorithm);
    auto res = drone.step();
    
    EXPECT_EQ(res.status, DroneStepStatus::Completed);
}

TEST_F(DroneControl, MovementFails_ReturnsError) {
    MappingStepCommand cmd;
    cmd.status = AlgorithmStatus::Working;
    cmd.movement = MovementCommand{MovementCommandType::Elevate, RotationDirection::Left, 0*deg, 10*cm};
    
    EXPECT_CALL(mapping_algorithm, nextStep(_, _)).WillOnce(Return(cmd));
    EXPECT_CALL(movement, elevate(_)).WillOnce(Return(MovementResult{false, "Engine failure"}));

    DroneControlImpl drone(drone_config, mission_config, lidar, gps, movement, output_map, mapping_algorithm);
    auto res = drone.step();
    
    EXPECT_EQ(res.status, DroneStepStatus::Error);
}

TEST_F(DroneControl, ScanResult_AppliedToMap) {
    MappingStepCommand cmd;
    cmd.status = AlgorithmStatus::Working;
    cmd.scan_orientation = Orientation{10*deg, 0*deg};
    
    EXPECT_CALL(mapping_algorithm, nextStep(_, _)).WillOnce(Return(cmd));
    
    LidarScanResult scan_res;
    EXPECT_CALL(lidar, scan(_)).WillOnce(Return(scan_res));

    DroneControlImpl drone(drone_config, mission_config, lidar, gps, movement, output_map, mapping_algorithm);
    auto res = drone.step();
    
    EXPECT_EQ(res.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, Hover_NoMovement) {
    MappingStepCommand cmd;
    cmd.status = AlgorithmStatus::Working;
    cmd.movement = MovementCommand{MovementCommandType::Hover, RotationDirection::Left, 0*deg, 0*cm};
    
    EXPECT_CALL(mapping_algorithm, nextStep(_, _)).WillOnce(Return(cmd));
    
    EXPECT_CALL(movement, advance(_)).Times(0);
    EXPECT_CALL(movement, rotate(_, _)).Times(0);
    EXPECT_CALL(movement, elevate(_)).Times(0);

    DroneControlImpl drone(drone_config, mission_config, lidar, gps, movement, output_map, mapping_algorithm);
    auto res = drone.step();
    
    EXPECT_EQ(res.status, DroneStepStatus::Continue);
}

TEST_F(DroneControl, StateTracking_StepIndexIncrementsByOne) {
    MappingStepCommand cmd;
    cmd.status = AlgorithmStatus::Working;
    cmd.movement = MovementCommand{MovementCommandType::Hover, RotationDirection::Left, 0*deg, 0*cm};
    
    EXPECT_CALL(mapping_algorithm, nextStep(_, _)).WillRepeatedly(Return(cmd));

    DroneControlImpl drone(drone_config, mission_config, lidar, gps, movement, output_map, mapping_algorithm);
    
    EXPECT_EQ(drone.state().step_index, 0);
    [[maybe_unused]] auto r1 = drone.step();
    EXPECT_EQ(drone.state().step_index, 1);
    [[maybe_unused]] auto r2 = drone.step();
    EXPECT_EQ(drone.state().step_index, 2);
}

TEST_F(DroneControl, StateTracking_StepIndexUnchangedOnQuery) {
    DroneControlImpl drone(drone_config, mission_config, lidar, gps, movement, output_map, mapping_algorithm);
    
    EXPECT_EQ(drone.state().step_index, 0);
    EXPECT_EQ(drone.state().step_index, 0);
    EXPECT_EQ(drone.state().step_index, 0);
}

} // namespace
