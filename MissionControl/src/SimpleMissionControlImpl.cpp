#include "SimpleMissionControlImpl.h"
#include <Common/MissionControlRegistration.h>
#include <UserCommon/Logger.h>
#include <fstream>

#include "DroneControlImpl.h"
#include <iostream>


namespace mission_control_330371063_324976703 {
using namespace common;

SimpleMissionControlImpl::SimpleMissionControlImpl(common::MissionControlDependencies deps)
    : mission_(deps.mission_config),
      output_map_(deps.output_map),
      output_map_file_(deps.output_map_file) {
      drone_control_ = std::make_unique<DroneControlImpl>(
          deps.drone_config, deps.mission_config, deps.lidar, deps.gps, deps.movement,
          deps.output_map, deps.mapping_algorithm);
}

common::types::MissionRunResult SimpleMissionControlImpl::runMission() {
    common::types::MissionRunResult result;
    result.status = common::types::MissionRunStatus::Completed;
    
    // Minimal error logger
    auto logError = [&](const std::string& msg) {
        std::filesystem::path error_log_path = output_map_file_.parent_path() / "error_log.txt";
        std::filesystem::create_directories(output_map_file_.parent_path());
        std::ofstream error_log(error_log_path, std::ios::app);
        if (error_log) {
            error_log << msg << "\n";
        }
    };

    for (std::size_t step = 0; step < mission_.max_steps; ++step) {
        if (step % 100 == 0) std::cout << "Step: " << step << std::endl;
        try {
            common::types::DroneStepResult step_result = drone_control_->step();

            if (step_result.status == common::types::DroneStepStatus::Completed) {
                result.status = common::types::MissionRunStatus::Completed;
                result.steps = step + 1;
                break;
            }

            if (step_result.status == common::types::DroneStepStatus::Error) {
                logError("Step " + std::to_string(step) + ": ERROR - " + step_result.message);
                result.status = common::types::MissionRunStatus::Error;
                result.steps = step + 1;
                result.errors.push_back(common::types::ErrorRef{
                    "DRONE_STEP_ERROR",
                    step_result.message
                });
                break;
            }
            
            result.steps = step + 1;
        } catch (const std::exception& e) {
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

    if (result.status == common::types::MissionRunStatus::Completed && result.steps >= mission_.max_steps) {
        result.status = common::types::MissionRunStatus::MaxSteps;
        result.steps = mission_.max_steps;
    }

    try {
        output_map_.save(output_map_file_);
    } catch (const std::exception& e) {
        logError("SAVE ERROR - " + std::string(e.what()));
        result.errors.push_back(common::types::ErrorRef{
            "MAP_SAVE_ERROR",
            e.what()
        });
    }

    return result;
}

REGISTER_MISSION_CONTROL(SimpleMissionControlImpl);

} // namespace mission_control_330371063_324976703
