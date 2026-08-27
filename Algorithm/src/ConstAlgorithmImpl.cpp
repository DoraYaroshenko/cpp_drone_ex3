#include "ConstAlgorithmImpl.h"
#include <Common/MappingAlgorithmRegistration.h>

namespace algorithm_330371063_324976703 {
using namespace common;

ConstAlgorithmImpl::ConstAlgorithmImpl(common::MappingAlgorithmDependencies dependencies)
    : IMappingAlgorithm(std::move(dependencies)) {
}

common::types::MappingStepCommand ConstAlgorithmImpl::nextStep(
    const common::types::DroneState& state,
    const common::types::LidarScanResult* latest_scan) {

    // Dummy algorithm: constantly advances forward without checking collisions
    common::types::MappingStepCommand command;
    command.scan_orientation = state.heading;
    command.movement = common::types::MovementCommand{
        common::types::MovementCommandType::Advance,
        common::types::RotationDirection::Right, // Irrelevant for advance
        common::HorizontalAngle(0 * common::deg),
        drone_config_.max_advance
    };

    return command;
}

REGISTER_MAPPING_ALGORITHM(ConstAlgorithmImpl);

} // namespace algorithm_330371063_324976703
