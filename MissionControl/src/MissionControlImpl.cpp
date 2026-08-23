#include "MissionControlImpl.h"
#include <UserCommon/CollisionUtils.h>
#include <UserCommon/Logger.h>
#include <Common/MissionControlRegistration.h>

#include <fstream>
#include <chrono>
#include <utility>

#include "DroneControlImpl.h"

namespace mission_control_330371063_324976703 {
using namespace common;

using namespace user_common_330371063_324976703;

MissionControlImpl_330371063_324976703::MissionControlImpl_330371063_324976703(common::MissionControlDependencies deps)
    : mission_(deps.mission_config),
      drone_(deps.drone_config),
      output_map_(deps.output_map),
      output_map_file_(deps.output_map_file) {
      drone_control_ = std::make_unique<DroneControlImpl>(
          deps.drone_config, deps.mission_config, deps.lidar, deps.gps, deps.movement,
          deps.output_map, deps.mapping_algorithm);
}

common::types::MissionRunResult MissionControlImpl_330371063_324976703::runMission() {
    common::types::MissionRunResult result;
    result.status = common::types::MissionRunStatus::Completed;

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
            common::types::DroneStepResult step_result = drone_control_->step();

            if (step_result.status == common::types::DroneStepStatus::Completed) {
                result.status = common::types::MissionRunStatus::Completed;
                result.steps = step + 1;
                break;
            }

            if (step_result.status == common::types::DroneStepStatus::Error) {
                // Log error immediately.
                logError("Step " + std::to_string(step) + ": ERROR - " + step_result.message);
                result.status = common::types::MissionRunStatus::Error;
                result.steps = step + 1;
                result.errors.push_back(common::types::ErrorRef{
                    "DRONE_STEP_ERROR",
                    step_result.message
                });
                break;
            }

            // DroneStepStatus::Continue — keep going.
            result.steps = step + 1;

            // Verify legality of the drone's position after movement
            common::types::DroneState current_state = drone_control_->state();
            
            // 1. Check if the drone went out of mission boundaries
            if (!CollisionUtils::isDroneFullyInBounds(output_map_, current_state.position, drone_.radius)) {
                logError("Step " + std::to_string(step) + ": ERROR - Drone flew completely out of the mission boundaries!");
                result.status = common::types::MissionRunStatus::Error;
                result.errors.push_back(common::types::ErrorRef{
                    "ILLEGAL_MOVEMENT_OUT_OF_BOUNDS",
                    "Drone's physical body is outside the configured mission boundaries."
                });
                break;
            }

        } catch (const std::exception& e) {
            // Log exception immediately.
            logError("Step " + std::to_string(step) + ": EXCEPTION - " + std::string(e.what()));
            result.status = common::types::MissionRunStatus::Error;
            result.steps = step + 1;
            result.errors.push_back(common::types::ErrorRef{
                "DRONE_EXCEPTION",
                e.what()
            });
            break;
        }
    }

    // If we exhausted max_steps without completing or erroring.
    if (result.status == common::types::MissionRunStatus::Completed && result.steps >= mission_.max_steps) {
        result.status = common::types::MissionRunStatus::MaxSteps;
        result.steps = mission_.max_steps;
    }

    // Save the output map.
    try {
        output_map_.save(output_map_file_);
    } catch (const std::exception& e) {
        logError("SAVE ERROR - " + std::string(e.what()));
        result.errors.push_back(common::types::ErrorRef{
            "MAP_SAVE_ERROR",
            e.what()
        });
    }

    Logger::close();
    return result;
}

REGISTER_MISSION_CONTROL(MissionControlImpl_330371063_324976703);

} // namespace mission_control_330371063_324976703
