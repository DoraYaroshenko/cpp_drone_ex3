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
#include <thread>
#include <atomic>
#include <vector>




namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;
namespace user_common_330371063_324976703 {}
using namespace user_common_330371063_324976703;

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory, int num_threads)
    : run_factory_(std::move(run_factory)), num_threads_(num_threads) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

// Helper struct to hold task parameters
struct RunTask {
    const types::SimulationConfigData* simulation;
    const types::MissionConfigData* mission;
    const types::DroneConfigData* drone;
    const types::LidarConfigData* lidar;
    std::filesystem::path run_output;
};

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::filesystem::path results_dir = output_path / "output_results";
    if (std::filesystem::exists(results_dir)) {
        std::filesystem::remove_all(results_dir);
    }
    std::filesystem::create_directories(results_dir);

    // Generate UTC timestamp.
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
#if defined(_MSC_VER)
    gmtime_s(&utc_tm, &time_t_now);
#else
    gmtime_r(&time_t_now, &utc_tm);
#endif
    std::ostringstream time_ss;
    time_ss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%SZ");

    std::vector<RunTask> tasks;
    int run_index = 0;

    // Cartesian product: simulations × missions × drones × lidars
    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        for (const types::MissionConfigData& mission : missions) {
            for (const types::DroneConfigData& drone : composition.drone_configs) {
                for (const types::LidarConfigData& lidar : composition.lidar_configs) {
                    std::filesystem::path run_output = results_dir / ("run_" + std::to_string(run_index));
                    std::filesystem::create_directories(run_output);

                    tasks.push_back({&simulation, &mission, &drone, &lidar, run_output});
                    ++run_index;
                }
            }
        }
    }

    std::vector<types::SimulationResult> runs(tasks.size());
    std::atomic<size_t> next_task{0};

    auto worker = [&]() {
        while (true) {
            size_t idx = next_task.fetch_add(1, std::memory_order_relaxed);
            if (idx >= tasks.size()) {
                break;
            }

            const auto& task = tasks[idx];
            try {
                std::unique_ptr<ISimulationRun> sim_run =
                    run_factory_->create(*(task.simulation), *(task.mission), *(task.drone), *(task.lidar), task.run_output / "output_map.npy");
                runs[idx] = sim_run->run();
            } catch (const std::exception& e) {
                // Write error log
                std::ofstream err_file(task.run_output / "error_log.txt", std::ios::app);
                if (err_file) {
                    err_file << "Simulation Run Error: " << e.what() << "\n";
                }

                types::SimulationResult error_result;
                error_result.simulation_config = *(task.simulation);
                error_result.mission_config = *(task.mission);
                error_result.mission_score = -1.0;
                error_result.mission_results.push_back(types::MissionRunResult{
                    types::MissionRunStatus::Error,
                    0,
                    {types::ErrorRef{"SIMULATION_RUN_ERROR", e.what()}}
                });
                runs[idx] = std::move(error_result);
            }
        }
    };

    if (num_threads_ <= 1) {
        // Run sequentially
        worker();
    } else {
        // Run concurrently. num_threads_ specifies the number of additional threads.
        int num_extra_threads = std::min<int>(num_threads_, static_cast<int>(tasks.size()));
        std::vector<std::jthread> threads;
        for (int i = 0; i < num_extra_threads; ++i) {
            threads.emplace_back(worker);
        }
        
        // Also do work on the main thread if needed
        worker();
        
        // jthreads automatically join on destruction.
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
