#include <gtest/gtest.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/Types.h>
#include "../TestHelpers.h"
#include "IntegrationFixture.h"

using namespace drone_mapper;
using namespace drone_mapper::types;
using namespace drone_mapper::tests;



TEST_F(Integration, RealAlgorithm_FullRun_CompletesWithPositiveScore) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));

    auto report = manager.run(comp, output_path);

    EXPECT_EQ(report.runs.size(), 1);
    EXPECT_GT(report.runs[0].mission_score, 0.0);
}

TEST_F(Integration, RealAlgorithm_FullRun_OutputMapFileCreated) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));

    auto report = manager.run(comp, output_path);

    ASSERT_EQ(report.runs.size(), 1);
    EXPECT_TRUE(std::filesystem::exists(report.runs[0].output_map_file));
}

TEST_F(Integration, RealAlgorithm_SimulationManagerReport_Structure) {
    auto factory = std::make_unique<SimulationRunFactoryImpl>();
    SimulationManager manager(std::move(factory));

    auto report = manager.run(comp, output_path);

    EXPECT_FALSE(report.generated_at_utc.empty());
    EXPECT_EQ(report.metric, "output_map_accuracy");
}
