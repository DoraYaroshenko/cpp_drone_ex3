#include <gtest/gtest.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/Types.h>
#include "../mocks/Mocks.h"
#include "../TestHelpers.h"
#include <filesystem>
#include <fstream>
#include <memory>

using namespace drone_mapper;
using namespace drone_mapper::types;
using namespace drone_mapper::tests;
using ::testing::_;
using ::testing::Return;
using ::testing::ByMove;
using ::testing::Invoke;
using ::testing::Throw;

namespace {
class SimulationManager : public ::testing::Test {
protected:
    SimulationCompositionData empty_composition;
    std::filesystem::path output_path = "test_output_dir";

    void SetUp() override {
        std::filesystem::remove_all(output_path);
    }
    void TearDown() override {
        std::filesystem::remove_all(output_path);
    }
};

TEST_F(SimulationManager, NullFactoryThrows) {
    EXPECT_THROW(drone_mapper::SimulationManager manager(nullptr), std::invalid_argument);
}

TEST_F(SimulationManager, EmptyComposition_ReturnsEmptyReport) {
    auto factory = std::make_unique<tests::MockSimulationRunFactory>();
    drone_mapper::SimulationManager manager(std::move(factory));

    auto report = manager.run(empty_composition, output_path);
    EXPECT_TRUE(report.runs.empty());
    EXPECT_EQ(report.error_score, -1);
}

TEST_F(SimulationManager, SingleRun_HappyPath) {
    auto factory = std::make_unique<tests::MockSimulationRunFactory>();
    
    auto mock_run = std::make_unique<tests::MockSimulationRun>();
    SimulationResult sim_res;
    sim_res.mission_score = 95.0;
    
    EXPECT_CALL(*mock_run, run()).WillOnce(Return(sim_res));
    EXPECT_CALL(*factory, create(_, _, _, _, _)).WillOnce(Return(ByMove(std::move(mock_run))));

    SimulationCompositionData comp;
    comp.simulation_mission_groups.push_back({makeTestSimulationConfig(), {makeTestMissionConfig()}});
    comp.drones = {makeTestDroneConfig()};
    comp.lidars = {makeTestLidarConfig()};

    drone_mapper::SimulationManager manager(std::move(factory));
    auto report = manager.run(comp, output_path);

    EXPECT_EQ(report.runs.size(), 1);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, 95.0);
}

TEST_F(SimulationManager, InvalidArguments_HandledGracefully) {
    auto factory = std::make_unique<tests::MockSimulationRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce(Throw(std::invalid_argument("Invalid bounds")));

    SimulationCompositionData comp;
    comp.simulation_mission_groups.push_back({makeTestSimulationConfig(), {makeTestMissionConfig()}});
    comp.drones = {makeTestDroneConfig()};
    comp.lidars = {makeTestLidarConfig()};

    drone_mapper::SimulationManager manager(std::move(factory));
    auto report = manager.run(comp, output_path);

    EXPECT_EQ(report.runs.size(), 1);
    EXPECT_EQ(report.runs[0].mission_score, -1.0);
}

TEST_F(SimulationManager, MultipleRuns_CartesianProduct) {
    auto factory = std::make_unique<tests::MockSimulationRunFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(8)
        .WillRepeatedly([](const types::SimulationConfigData&, const types::MissionConfigData&, const types::DroneConfigData&, const types::LidarConfigData&, const std::filesystem::path&) {
            auto run = std::make_unique<tests::MockSimulationRun>();
            EXPECT_CALL(*run, run()).WillRepeatedly(Return(types::SimulationResult{}));
            return run;
        });

    SimulationCompositionData comp;
    comp.simulation_mission_groups.push_back({makeTestSimulationConfig(), {makeTestMissionConfig()}});
    comp.simulation_mission_groups.push_back({makeTestSimulationConfig(), {makeTestMissionConfig()}});
    comp.drones = {makeTestDroneConfig(), makeTestDroneConfig()};
    comp.lidars = {makeTestLidarConfig(), makeTestLidarConfig()};

    drone_mapper::SimulationManager manager(std::move(factory));
    auto report = manager.run(comp, output_path);

    EXPECT_EQ(report.runs.size(), 8);
}

TEST_F(SimulationManager, RunThrowsException_RecordsErrorResult) {
    auto factory = std::make_unique<tests::MockSimulationRunFactory>();
    auto mock_run = std::make_unique<tests::MockSimulationRun>();
    EXPECT_CALL(*mock_run, run()).WillOnce(Throw(std::runtime_error("Crash")));

    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce(Return(ByMove(std::move(mock_run))));

    SimulationCompositionData comp;
    comp.simulation_mission_groups.push_back({makeTestSimulationConfig(), {makeTestMissionConfig()}});
    comp.drones = {makeTestDroneConfig()};
    comp.lidars = {makeTestLidarConfig()};

    drone_mapper::SimulationManager manager(std::move(factory));
    auto report = manager.run(comp, output_path);

    EXPECT_EQ(report.runs.size(), 1);
    EXPECT_EQ(report.runs[0].mission_score, -1.0);
}

TEST_F(SimulationManager, ReportMetadata) {
    auto factory = std::make_unique<tests::MockSimulationRunFactory>();
    drone_mapper::SimulationManager manager(std::move(factory));
    auto report = manager.run(empty_composition, output_path);
    EXPECT_FALSE(report.generated_at_utc.empty());
    EXPECT_EQ(report.metric, "output_map_accuracy");
}

TEST_F(SimulationManager, OutputDirectoryCleaned) {
    std::filesystem::path results_dir = output_path / "output_results";
    std::filesystem::create_directories(results_dir);
    std::filesystem::path dummy = results_dir / "dummy.txt";
    {
        std::ofstream ofs(dummy);
        ofs << "test";
    }
    
    auto factory = std::make_unique<tests::MockSimulationRunFactory>();
    auto mock_run = std::make_unique<tests::MockSimulationRun>();
    EXPECT_CALL(*mock_run, run()).WillOnce(Return(SimulationResult{}));
    EXPECT_CALL(*factory, create(_, _, _, _, _)).WillOnce(Return(ByMove(std::move(mock_run))));
    drone_mapper::SimulationManager manager(std::move(factory));
        SimulationCompositionData dummy_composition;
    dummy_composition.simulation_mission_groups.push_back({makeTestSimulationConfig(), {makeTestMissionConfig()}});
    dummy_composition.drones = {makeTestDroneConfig()};
    dummy_composition.lidars = {makeTestLidarConfig()};
    [[maybe_unused]] auto res = manager.run(dummy_composition, output_path);

    EXPECT_FALSE(std::filesystem::exists(dummy));
}

} // namespace
