#include "Algorithm/SimpleSweepAlgorithmImpl.h"
#include <Common/MappingAlgorithmRegistration.h>
#include <UserCommon/CollisionUtils.h>

#include <cmath>

namespace algorithm_330371063_324976703 {
using namespace common;
using namespace user_common_330371063_324976703;

SimpleSweepAlgorithmImpl::SimpleSweepAlgorithmImpl(common::MappingAlgorithmDependencies dependencies)
    : IMappingAlgorithm(std::move(dependencies)) {
}

common::types::MappingStepCommand SimpleSweepAlgorithmImpl::nextStep(
    const common::types::DroneState& state,
    const common::types::LidarScanResult* latest_scan) {

    common::types::MappingStepCommand command;
    command.scan_orientation = state.heading;

    if (rotate_next_) {
        command.movement = common::types::MovementCommand{
            common::types::MovementCommandType::Rotate,
            common::types::RotationDirection::Right,
            common::HorizontalAngle(90 * common::deg),
            common::PhysicalLength(0 * common::cm)
        };
        rotate_next_ = false;
        return command;
    }

    bool blocked = false;
    if (latest_scan != nullptr) {
        for (const auto& measure : *latest_scan) {
            if (measure.distance < drone_config_.max_advance + drone_config_.radius) {
                // Approximate check if obstacle is straight ahead (very simple logic for dummy algorithm)
                if (std::abs(measure.angle.horizontal.numerical_value_in(common::deg) - 
                             state.heading.horizontal.numerical_value_in(common::deg)) < 15.0) {
                    blocked = true;
                    break;
                }
            }
        }
    }

    if (blocked) {
        rotate_next_ = true;
        command.movement = common::types::MovementCommand{
            common::types::MovementCommandType::Rotate,
            common::types::RotationDirection::Right,
            common::HorizontalAngle(90 * common::deg),
            common::PhysicalLength(0 * common::cm)
        };
        rotate_next_ = false;
    } else {
        command.movement = common::types::MovementCommand{
            common::types::MovementCommandType::Advance,
            common::types::RotationDirection::Right,
            common::HorizontalAngle(0 * common::deg),
            drone_config_.max_advance
        };
    }

    return command;
}

REGISTER_MAPPING_ALGORITHM(SimpleSweepAlgorithmImpl);

} // namespace algorithm_330371063_324976703
