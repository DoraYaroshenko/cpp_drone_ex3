#include "Algorithm/CyclingAlgorithmImpl.h"
#include <Common/MappingAlgorithmRegistration.h>

namespace algorithm_330371063_324976703 {
using namespace common;

CyclingAlgorithmImpl::CyclingAlgorithmImpl(common::MappingAlgorithmDependencies dependencies)
    : IMappingAlgorithm(std::move(dependencies)) {
}

common::types::MappingStepCommand CyclingAlgorithmImpl::nextStep(
    const common::types::DroneState& state,
    const common::types::LidarScanResult* latest_scan) {

    // Cycles between advance and rotate 90 deg right
    common::types::MappingStepCommand command;
    command.scan_orientation = state.heading;

    if (step_count_ % 2 == 0) {
        command.movement = common::types::MovementCommand{
            common::types::MovementCommandType::Advance,
            common::types::RotationDirection::Right,
            common::HorizontalAngle(0 * common::deg),
            drone_config_.max_advance
        };
    } else {
        command.movement = common::types::MovementCommand{
            common::types::MovementCommandType::Rotate,
            common::types::RotationDirection::Right,
            common::HorizontalAngle(90 * common::deg),
            common::PhysicalLength(0 * common::cm)
        };
    }

    step_count_++;
    return command;
}

REGISTER_MAPPING_ALGORITHM(CyclingAlgorithmImpl);

} // namespace algorithm_330371063_324976703
