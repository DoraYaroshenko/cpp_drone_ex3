#pragma once
#include <gtest/gtest.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/Types.h>
#include <filesystem>
#include "../TestHelpers.h"

class Integration : public ::testing::Test {
protected:
    drone_mapper::types::SimulationCompositionData comp;
    std::filesystem::path output_path = "test_integration_output";

    void SetUp() override {
        std::filesystem::remove_all(output_path);
        std::filesystem::create_directories(output_path);

        drone_mapper::tests::saveTestNpyArray("test_hidden_integration.npy", 10, 10, 10, 0);
        
        comp.simulation_mission_groups.push_back({
            drone_mapper::tests::makeTestSimulationConfig(), 
            {drone_mapper::tests::makeTestMissionConfig()}
        });
        std::get<0>(comp.simulation_mission_groups[0]).map_filename = "test_hidden_integration.npy";
        
        comp.drones = {drone_mapper::tests::makeTestDroneConfig()};
        comp.lidars = {drone_mapper::tests::makeTestLidarConfig()};
    }

    void TearDown() override {
        std::filesystem::remove_all(output_path);
        std::filesystem::remove("test_hidden_integration.npy");
    }
};
