#include <gtest/gtest.h>
#include <drone_mapper/SimulationRunImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/Types.h>
#include "../mocks/Mocks.h"
#include "../TestHelpers.h"
#include <cmath>

using namespace drone_mapper;
using namespace drone_mapper::types;
using namespace drone_mapper::tests;
using ::testing::_;
using ::testing::Return;
using ::testing::NiceMock;
using ::testing::Throw;

namespace {
class SimulationRun : public ::testing::Test {
protected:
    SimulationConfigData sim_config = makeTestSimulationConfig();
    MissionConfigData mission_config = makeTestMissionConfig();
    std::filesystem::path output_map_file = "test_output.npy";

    std::unique_ptr<tests::MockMap3D> hidden_map = std::make_unique<NiceMock<tests::MockMap3D>>();
    std::unique_ptr<tests::MockMutableMap3D> output_map = std::make_unique<NiceMock<tests::MockMutableMap3D>>();
    std::unique_ptr<tests::MockGPS> gps = std::make_unique<NiceMock<tests::MockGPS>>();
    std::unique_ptr<tests::MockDroneMovement> movement = std::make_unique<NiceMock<tests::MockDroneMovement>>();
    std::unique_ptr<tests::MockLidar> lidar = std::make_unique<NiceMock<tests::MockLidar>>();
    std::unique_ptr<tests::MockMappingAlgorithm> algorithm;
    std::unique_ptr<tests::MockDroneControl> drone_control = std::make_unique<NiceMock<tests::MockDroneControl>>();
    std::unique_ptr<tests::MockMissionControl> mission_control = std::make_unique<NiceMock<tests::MockMissionControl>>();

    void SetUp() override {
        algorithm = std::make_unique<tests::MockMappingAlgorithm>(makeTestMissionConfig(), makeTestLidarConfig(), makeTestDroneConfig(), *output_map);
        
        MapConfig cfg = makeTestMapConfig();
        ON_CALL(*hidden_map, getMapConfig()).WillByDefault(Return(cfg));
        ON_CALL(*output_map, getMapConfig()).WillByDefault(Return(cfg));
        ON_CALL(*hidden_map, isInBounds(_)).WillByDefault(Return(true));
        ON_CALL(*output_map, isInBounds(_)).WillByDefault(Return(true));
        
        // Return matching values so comparison doesn't return 0.
        // Or actually, MockMap3D returns 0 (Unmapped) by default.
        // We will just let them both return Unmapped, so score is 100%.
    }
};

TEST_F(SimulationRun, NullDependencyThrows) {
    EXPECT_THROW(SimulationRunImpl(nullptr, std::move(output_map), std::move(gps), std::move(movement), std::move(lidar), std::move(algorithm), std::move(drone_control), std::move(mission_control), sim_config, mission_config, output_map_file), std::invalid_argument);
}

TEST_F(SimulationRun, HappyPath_MissionCompletedWithScore) {
    MissionRunResult run_res{MissionRunStatus::Completed, 100, {}};
    EXPECT_CALL(*mission_control, runMission()).WillOnce(Return(run_res));
    
    saveTestNpyArray("test_hidden.npy", 10, 10, 10, 1);
    saveTestNpyArray("test_output.npy", 10, 10, 10, 1);
    sim_config.map_filename = "test_hidden.npy";
    output_map_file = "test_output.npy";
    
    SimulationRunImpl run(std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement), std::move(lidar), std::move(algorithm), std::move(drone_control), std::move(mission_control), sim_config, mission_config, output_map_file);
    
    auto result = run.run();
    
    EXPECT_EQ(result.mission_results[0].status, MissionRunStatus::Completed);
    EXPECT_DOUBLE_EQ(result.mission_score, 100.0);
    
    std::filesystem::remove("test_hidden.npy");
    std::filesystem::remove("test_output.npy");
}

TEST_F(SimulationRun, MissionError_ScoreIsMinusOne) {
    MissionRunResult run_res{MissionRunStatus::Error, 10, {ErrorRef{"TEST_ERR", "Msg"}}};
    EXPECT_CALL(*mission_control, runMission()).WillOnce(Return(run_res));
    
    saveTestNpyArray("test_hidden.npy", 10, 10, 10, 1);
    saveTestNpyArray("test_output.npy", 10, 10, 10, 1);
    sim_config.map_filename = "test_hidden.npy";
    output_map_file = "test_output.npy";

    SimulationRunImpl run(std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement), std::move(lidar), std::move(algorithm), std::move(drone_control), std::move(mission_control), sim_config, mission_config, output_map_file);
    auto result = run.run();
    
    EXPECT_EQ(result.mission_results[0].status, MissionRunStatus::Error);
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
    
    std::filesystem::remove("test_hidden.npy");
    std::filesystem::remove("test_output.npy");
}

TEST_F(SimulationRun, MissionException_CaughtAndRecorded) {
    EXPECT_CALL(*mission_control, runMission()).WillOnce(Throw(std::runtime_error("Simulated Crash")));
    
    saveTestNpyArray("test_hidden.npy", 10, 10, 10, 1);
    saveTestNpyArray("test_output.npy", 10, 10, 10, 1);
    sim_config.map_filename = "test_hidden.npy";
    output_map_file = "test_output.npy";

    SimulationRunImpl run(std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement), std::move(lidar), std::move(algorithm), std::move(drone_control), std::move(mission_control), sim_config, mission_config, output_map_file);
    auto result = run.run();
    
    EXPECT_EQ(result.mission_results[0].status, MissionRunStatus::Error);
    EXPECT_EQ(result.mission_results[0].errors.front().code, "MISSION_EXCEPTION");
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);

    std::filesystem::remove("test_hidden.npy");
    std::filesystem::remove("test_output.npy");
}

TEST_F(SimulationRun, ResolutionFactor_Accepted) {
    mission_config.output_mapping_resolution_factor = 1.0;
    saveTestNpyArray("test_hidden.npy", 10, 10, 10, 1);
    saveTestNpyArray("test_output.npy", 10, 10, 10, 1);
    sim_config.map_filename = "test_hidden.npy";
    output_map_file = "test_output.npy";
    EXPECT_CALL(*mission_control, runMission()).WillOnce(Return(MissionRunResult{MissionRunStatus::Completed, 0, {}}));

    SimulationRunImpl run(std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement), std::move(lidar), std::move(algorithm), std::move(drone_control), std::move(mission_control), sim_config, mission_config, output_map_file);
    auto result = run.run();
    
    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Accepted);
    
    std::filesystem::remove("test_hidden.npy");
    std::filesystem::remove("test_output.npy");
}

TEST_F(SimulationRun, ResolutionFactor_Ignored) {
    mission_config.output_mapping_resolution_factor = 2.0;
    saveTestNpyArray("test_hidden.npy", 10, 10, 10, 1);
    saveTestNpyArray("test_output.npy", 10, 10, 10, 1);
    sim_config.map_filename = "test_hidden.npy";
    output_map_file = "test_output.npy";
    EXPECT_CALL(*mission_control, runMission()).WillOnce(Return(MissionRunResult{MissionRunStatus::Completed, 0, {}}));

    SimulationRunImpl run(std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement), std::move(lidar), std::move(algorithm), std::move(drone_control), std::move(mission_control), sim_config, mission_config, output_map_file);
    auto result = run.run();
    
    EXPECT_EQ(result.resolution_request_status, ResolutionRequestStatus::Ignored);
    
    std::filesystem::remove("test_hidden.npy");
    std::filesystem::remove("test_output.npy");
}

TEST(MockGPSTest, InitialPositionAndHeading) {
    Position3D pos{10*x_extent[cm], 20*y_extent[cm], 30*z_extent[cm]};
    Orientation head{45*deg, 0*altitude_angle[deg]};
    drone_mapper::MockGPS mock_gps(pos, head, 1.0*cm);
    
    EXPECT_EQ(mock_gps.position().x.numerical_value_in(cm), 10.0);
    EXPECT_EQ(mock_gps.position().y.numerical_value_in(cm), 20.0);
    EXPECT_EQ(mock_gps.position().z.numerical_value_in(cm), 30.0);
    EXPECT_EQ(mock_gps.heading().horizontal.numerical_value_in(deg), 45.0);
    EXPECT_EQ(mock_gps.heading().altitude.numerical_value_in(deg), 0.0);
}

TEST(MockGPSTest, SetPositionSetHeading) {
    Position3D pos{0*x_extent[cm], 0*y_extent[cm], 0*z_extent[cm]};
    Orientation head{0*deg, 0*altitude_angle[deg]};
    drone_mapper::MockGPS mock_gps(pos, head, 1.0*cm);
    
    Position3D new_pos{100*x_extent[cm], 200*y_extent[cm], 300*z_extent[cm]};
    Orientation new_head{90*deg, 0*altitude_angle[deg]};
    
    mock_gps.setPosition(new_pos);
    mock_gps.setHeading(new_head);
    
    EXPECT_EQ(mock_gps.position().x.numerical_value_in(cm), 100.0);
    EXPECT_EQ(mock_gps.heading().horizontal.numerical_value_in(deg), 90.0);
}

TEST(MockMovementTest, Advance_UpdatesPosition) {
    Position3D pos{0*x_extent[cm], 0*y_extent[cm], 0*z_extent[cm]};
    Orientation head{0*deg, 0*altitude_angle[deg]};
    drone_mapper::MockGPS mock_gps(pos, head, 1.0*cm);
    drone_mapper::MockMovement mov(mock_gps);
    
    mov.advance(10*cm);
    
    EXPECT_DOUBLE_EQ(mock_gps.position().x.numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(mock_gps.position().y.numerical_value_in(cm), 0.0);
}

TEST(MockMovementTest, Advance_DifferentHeading) {
    Position3D pos{0*x_extent[cm], 0*y_extent[cm], 0*z_extent[cm]};
    Orientation head{90*deg, 0*altitude_angle[deg]};
    drone_mapper::MockGPS mock_gps(pos, head, 1.0*cm);
    drone_mapper::MockMovement mov(mock_gps);
    
    mov.advance(10*cm);
    
    EXPECT_NEAR(mock_gps.position().x.numerical_value_in(cm), 0.0, 1e-5);
    EXPECT_DOUBLE_EQ(mock_gps.position().y.numerical_value_in(cm), 10.0);
}

TEST(MockMovementTest, Advance_DiagonalHeading) {
    Position3D pos{0*x_extent[cm], 0*y_extent[cm], 0*z_extent[cm]};
    Orientation head{45*deg, 0*altitude_angle[deg]};
    drone_mapper::MockGPS mock_gps(pos, head, 1.0*cm);
    drone_mapper::MockMovement mov(mock_gps);
    
    mov.advance(10*cm);
    
    double expected = 10.0 * std::cos(M_PI / 4.0);
    EXPECT_NEAR(mock_gps.position().x.numerical_value_in(cm), expected, 1e-5);
    EXPECT_NEAR(mock_gps.position().y.numerical_value_in(cm), expected, 1e-5);
}

TEST(MockMovementTest, Rotate_LeavesPositionUnchanged) {
    Position3D pos{10*x_extent[cm], 20*y_extent[cm], 30*z_extent[cm]};
    Orientation head{0*deg, 0*altitude_angle[deg]};
    drone_mapper::MockGPS mock_gps(pos, head, 1.0*cm);
    drone_mapper::MockMovement mov(mock_gps);
    
    mov.rotate(RotationDirection::Left, 45*deg);
    
    EXPECT_DOUBLE_EQ(mock_gps.position().x.numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(mock_gps.position().y.numerical_value_in(cm), 20.0);
    EXPECT_DOUBLE_EQ(mock_gps.position().z.numerical_value_in(cm), 30.0);
}

TEST(MockMovementTest, Advance_LeavesAltitudeUnchanged) {
    Position3D pos{0*x_extent[cm], 0*y_extent[cm], 30*z_extent[cm]};
    Orientation head{0*deg, 0*altitude_angle[deg]};
    drone_mapper::MockGPS mock_gps(pos, head, 1.0*cm);
    drone_mapper::MockMovement mov(mock_gps);
    
    mov.advance(10*cm);
    
    EXPECT_DOUBLE_EQ(mock_gps.position().z.numerical_value_in(cm), 30.0);
}

TEST(MockMovementTest, Elevate_LeavesXYUnchanged) {
    Position3D pos{10*x_extent[cm], 20*y_extent[cm], 30*z_extent[cm]};
    Orientation head{0*deg, 0*altitude_angle[deg]};
    drone_mapper::MockGPS mock_gps(pos, head, 1.0*cm);
    drone_mapper::MockMovement mov(mock_gps);
    
    mov.elevate(15*cm);
    
    EXPECT_DOUBLE_EQ(mock_gps.position().x.numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(mock_gps.position().y.numerical_value_in(cm), 20.0);
    EXPECT_DOUBLE_EQ(mock_gps.position().z.numerical_value_in(cm), 45.0);
}

TEST(MockMovementTest, Rotate_LeftRight) {
    Position3D pos{0*x_extent[cm], 0*y_extent[cm], 0*z_extent[cm]};
    Orientation head{0*deg, 0*altitude_angle[deg]};
    drone_mapper::MockGPS mock_gps(pos, head, 1.0*cm);
    drone_mapper::MockMovement mov(mock_gps);
    
    mov.rotate(RotationDirection::Left, 45*deg); 
    auto az1 = mock_gps.heading().horizontal.numerical_value_in(deg);
    EXPECT_DOUBLE_EQ(az1, 45.0); 
    
    mov.rotate(RotationDirection::Right, 90*deg);
    auto az2 = mock_gps.heading().horizontal.numerical_value_in(deg);
    EXPECT_DOUBLE_EQ(az2, -45.0);
}

} // namespace
