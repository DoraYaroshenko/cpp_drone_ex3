#include "DroneControlImpl.h"
#include <UserCommon/ScanResultToVoxels.h>
#include <UserCommon/Logger.h>

#include <utility>
#include <string>

namespace mission_control_330371063_324976703 {
using namespace common;

using namespace user_common_330371063_324976703;

DroneControlImpl::DroneControlImpl(common::types::DroneConfigData drone,
                                   common::types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

common::types::DroneStepResult DroneControlImpl::step() {
    Logger::setStep(step_index_);

    // 1. Build current state.
    common::types::DroneState current_state = state();

    // 2. Get the latest scan result pointer (null on first call).
    static thread_local const common::types::LidarScanResult* last_scan_ptr = nullptr;
    static thread_local common::types::LidarScanResult last_scan_storage;

    // 3. Obtain command from algorithm or pending state
    common::types::MappingStepCommand cmd;
    if (pending_command_.has_value()) {
        cmd = pending_command_.value();
    } else {
        cmd = mapping_algorithm_.nextStep(current_state, last_scan_ptr);
        last_scan_ptr = nullptr; // Reset scan pointer

        if (cmd.status == common::types::AlgorithmStatus::Finished ||
            cmd.status == common::types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            return common::types::DroneStepResult{common::types::DroneStepStatus::Completed, "Mapping complete."};
        }
    }

    bool is_final_chunk = true;

    // 4. Execute movement command chunk
    if (cmd.movement.has_value()) {
        auto& move = cmd.movement.value();
        common::types::MovementResult result{true, {}};

        switch (move.type) {
            case common::types::MovementCommandType::Rotate: {
                double max_rot = drone_.max_rotate.numerical_value_in(deg);
                double cmd_angle = move.angle.numerical_value_in(deg);
                double chunk = std::min(std::abs(cmd_angle), max_rot);
                if (cmd_angle < 0) chunk = -chunk;

                result = movement_.rotate(move.rotation, HorizontalAngle(chunk * deg));
                
                double remaining = cmd_angle - chunk;
                if (std::abs(remaining) > 0.001) {
                    move.angle = HorizontalAngle(remaining * deg);
                    is_final_chunk = false;
                }
                break;
            }
            case common::types::MovementCommandType::Advance: {
                double max_adv = drone_.max_advance.numerical_value_in(cm);
                double cmd_dist = move.distance.numerical_value_in(cm);
                double chunk = std::min(cmd_dist, max_adv);

                result = movement_.advance(PhysicalLength(chunk * cm));
                
                double remaining = cmd_dist - chunk;
                if (remaining > 0.001) {
                    move.distance = PhysicalLength(remaining * cm);
                    is_final_chunk = false;
                }
                break;
            }
            case common::types::MovementCommandType::Elevate: {
                double max_elev = drone_.max_elevate.numerical_value_in(cm);
                double cmd_dist = move.distance.numerical_value_in(cm);
                double chunk = std::min(std::abs(cmd_dist), max_elev);
                if (cmd_dist < 0) chunk = -chunk;

                result = movement_.elevate(PhysicalLength(chunk * cm));
                
                double remaining = cmd_dist - chunk;
                if (std::abs(remaining) > 0.001) {
                    move.distance = PhysicalLength(remaining * cm);
                    is_final_chunk = false;
                }
                break;
            }
            case common::types::MovementCommandType::Hover:
                break;
        }

        if (!result) {
            pending_command_.reset();
            return common::types::DroneStepResult{
                common::types::DroneStepStatus::Error,
                "Movement failed: " + result.message
            };
        }
    }

    // 5. Execute scan if requested, but ONLY if this is the final movement chunk
    if (is_final_chunk) {
        if (cmd.scan_orientation.has_value()) {
            const Orientation& scan_orient = cmd.scan_orientation.value();
            last_scan_storage = lidar_.scan(scan_orient);
            last_scan_ptr = &last_scan_storage;

            common::types::LidarConfigData lidar_config = lidar_.config();
            ScanResultToVoxels::applyToMap(
                output_map_,
                gps_.position(),
                gps_.heading(),
                last_scan_storage,
                lidar_config
            );
        }
        pending_command_.reset();
    } else {
        pending_command_ = cmd;
    }

    // Log the movement
    std::string action_str = "hover";
    if (cmd.movement.has_value()) {
        switch (cmd.movement.value().type) {
            case common::types::MovementCommandType::Rotate:  action_str = "rotate"; break;
            case common::types::MovementCommandType::Advance: action_str = "advance"; break;
            case common::types::MovementCommandType::Elevate: action_str = "elevate"; break;
            case common::types::MovementCommandType::Hover:   action_str = "hover"; break;
        }
    }
    
    common::types::DroneState end_state = state();
    Logger::logMovement(
        end_state.position.x.numerical_value_in(cm),
        end_state.position.y.numerical_value_in(cm),
        end_state.position.z.numerical_value_in(cm),
        end_state.heading.horizontal.numerical_value_in(deg),
        end_state.heading.altitude.numerical_value_in(deg),
        action_str
    );

    // If we performed a scan, log the scan rays
    if (is_final_chunk && cmd.scan_orientation.has_value() && last_scan_ptr != nullptr) {
        for (const auto& hit : *last_scan_ptr) {
            Logger::logScan(
                hit.angle.horizontal.numerical_value_in(deg),
                hit.angle.altitude.numerical_value_in(deg)
            );
        }
    }

    // 6. Increment step counter.
    ++step_index_;

    return common::types::DroneStepResult{common::types::DroneStepStatus::Continue, {}};
}

common::types::DroneState DroneControlImpl::state() const {
    return common::types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace mission_control_330371063_324976703
