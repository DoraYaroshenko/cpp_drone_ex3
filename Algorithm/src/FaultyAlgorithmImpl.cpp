#include "Algorithm/FaultyAlgorithmImpl.h"
#include <Common/MappingAlgorithmRegistration.h>

namespace algorithm_330371063_324976703 {

common::types::MappingStepCommand FaultyAlgorithmImpl::nextStep(
    const common::types::DroneState& state,
    const common::types::LidarScanResult* latest_scan) {

    // Faulty behavior: Ignore everything and advance unconditionally by max_advance.
    // This will eventually result in a collision or going out of bounds.
    common::types::MappingStepCommand command;
    command.movement = common::types::MovementCommand{
        common::types::MovementCommandType::Advance,
        common::types::RotationDirection::Right,
        common::HorizontalAngle(0 * common::deg),
        drone_config_.max_advance
    };

    return command;
}

REGISTER_MAPPING_ALGORITHM(FaultyAlgorithmImpl);

} // namespace algorithm_330371063_324976703
