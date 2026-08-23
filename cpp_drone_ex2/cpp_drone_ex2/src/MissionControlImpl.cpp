#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/CollisionUtils.h>
#include <drone_mapper/Logger.h>

#include <fstream>
#include <chrono>
#include <utility>

namespace drone_mapper {

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

types::MissionRunResult MissionControlImpl::runMission() {
    types::MissionRunResult result;
    result.status = types::MissionRunStatus::Completed;

    // Initialize the logger for this mission run
    Logger::init((output_map_file_.parent_path() / "drone_logs.jsonl").string());

    auto logError = [&](const std::string& msg) {
        std::filesystem::path error_log_path = output_map_file_.parent_path() / "error_log.txt";
        std::filesystem::create_directories(output_map_file_.parent_path());
        std::ofstream error_log(error_log_path, std::ios::app);
        if (error_log) {
            error_log << msg << "\n";
        }
    };

    for (std::size_t step = 0; step < mission_.max_steps; ++step) {
        try {
            types::DroneStepResult step_result = drone_control_.step();

            if (step_result.status == types::DroneStepStatus::Completed) {
                result.status = types::MissionRunStatus::Completed;
                result.steps = step + 1;
                break;
            }

            if (step_result.status == types::DroneStepStatus::Error) {
                // Log error immediately.
                logError("Step " + std::to_string(step) + ": ERROR - " + step_result.message);
                result.status = types::MissionRunStatus::Error;
                result.steps = step + 1;
                result.errors.push_back(types::ErrorRef{
                    "DRONE_STEP_ERROR",
                    step_result.message
                });
                break;
            }

            // DroneStepStatus::Continue — keep going.
            result.steps = step + 1;

            // Verify legality of the drone's position after movement
            types::DroneState current_state = drone_control_.state();
            
            // 1. Check if the drone crashed into obstacles
            if (CollisionUtils::isDroneColliding(hidden_map_, current_state.position, drone_.radius)) {
                logError("Step " + std::to_string(step) + ": ERROR - Drone crashed into an obstacle!");
                result.status = types::MissionRunStatus::Error;
                result.errors.push_back(types::ErrorRef{
                    "ILLEGAL_MOVEMENT_COLLISION",
                    "Drone intersects with an occupied voxel in the hidden map."
                });
                break;
            }

            // 2. Check if the drone went out of mission boundaries
            if (!CollisionUtils::isDroneFullyInBounds(output_map_, current_state.position, drone_.radius)) {
                logError("Step " + std::to_string(step) + ": ERROR - Drone flew completely out of the mission boundaries!");
                result.status = types::MissionRunStatus::Error;
                result.errors.push_back(types::ErrorRef{
                    "ILLEGAL_MOVEMENT_OUT_OF_BOUNDS",
                    "Drone's physical body is outside the configured mission boundaries."
                });
                break;
            }

        } catch (const std::exception& e) {
            // Log exception immediately.
            logError("Step " + std::to_string(step) + ": EXCEPTION - " + std::string(e.what()));
            result.status = types::MissionRunStatus::Error;
            result.steps = step + 1;
            result.errors.push_back(types::ErrorRef{
                "DRONE_EXCEPTION",
                e.what()
            });
            break;
        }
    }

    // If we exhausted max_steps without completing or erroring.
    if (result.status == types::MissionRunStatus::Completed && result.steps >= mission_.max_steps) {
        result.status = types::MissionRunStatus::MaxSteps;
        result.steps = mission_.max_steps;
    }

    // Save the output map.
    try {
        output_map_.save(output_map_file_);
    } catch (const std::exception& e) {
        logError("SAVE ERROR - " + std::string(e.what()));
        result.errors.push_back(types::ErrorRef{
            "MAP_SAVE_ERROR",
            e.what()
        });
    }

    Logger::close();
    return result;
}

} // namespace drone_mapper
