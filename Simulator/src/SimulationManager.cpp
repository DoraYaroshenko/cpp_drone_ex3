#include <Simulator/SimulationManager.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <fstream>
#include <filesystem>




namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;
namespace user_common_330371063_324976703 {}
using namespace user_common_330371063_324976703;

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;

    std::filesystem::path results_dir = output_path / "output_results";
    if (std::filesystem::exists(results_dir)) {
        std::filesystem::remove_all(results_dir);
    }

    // Generate UTC timestamp.
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
    gmtime_r(&time_t_now, &utc_tm);
    std::ostringstream time_ss;
    time_ss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%SZ");

    int run_index = 0;

    // Cartesian product: simulations × missions × drones × lidars
    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        for (const types::MissionConfigData& mission : missions) {
            for (const types::DroneConfigData& drone : composition.drone_configs) {
                for (const types::LidarConfigData& lidar : composition.lidar_configs) {
                    // Create the output_results subdirectory inside output_path
                    std::filesystem::path results_dir = output_path / "output_results";
                    std::filesystem::create_directories(results_dir);

                    // Create a unique output path for each run.
                    std::filesystem::path run_output = results_dir / ("run_" + std::to_string(run_index));
                    std::filesystem::create_directories(run_output);

                    try {
                        std::unique_ptr<ISimulationRun> sim_run =
                            run_factory_->create(simulation, mission, drone, lidar, run_output);
                        types::SimulationResult result = sim_run->run();
                        runs.push_back(std::move(result));
                    } catch (const std::exception& e) {
                        // Write error log
                        std::filesystem::create_directories(run_output);
                        std::ofstream err_file(run_output / "error_log.txt", std::ios::app);
                        if (err_file) {
                            err_file << "Simulation Run Error: " << e.what() << "\n";
                        }

                        // Error in this run — record with score -1 and continue.
                        types::SimulationResult error_result;
                        error_result.simulation_config = simulation;
                        error_result.mission_config = mission;
                        error_result.mission_score = -1.0;
                        error_result.mission_results.push_back(types::MissionRunResult{
                            types::MissionRunStatus::Error,
                            0,
                            {types::ErrorRef{"SIMULATION_RUN_ERROR", e.what()}}
                        });
                        runs.push_back(std::move(error_result));
                    }

                    ++run_index;
                }
            }
        }
    }

    return types::SimulationManagerReport{
        composition.composition_file,
        time_ss.str(),
        "output_map_accuracy",
        {0.0, 100.0},
        -1,
        std::move(runs)
    };
}




} // namespace simulator
