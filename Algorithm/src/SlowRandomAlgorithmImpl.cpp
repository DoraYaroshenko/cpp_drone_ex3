#include "SlowRandomAlgorithmImpl.h"
#include <Common/MappingAlgorithmRegistration.h>

#include <chrono>
#include <thread>
#include <random>

namespace algorithm_330371063_324976703 {
using namespace common;

SlowRandomAlgorithmImpl::SlowRandomAlgorithmImpl(common::MappingAlgorithmDependencies dependencies)
    : IMappingAlgorithm(std::move(dependencies)) {
}

common::types::MappingStepCommand SlowRandomAlgorithmImpl::nextStep(
    const common::types::DroneState& state,
    const common::types::LidarScanResult* latest_scan) {

    // Simulating slow algorithm wait
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    static std::mt19937 gen(1337);
    std::uniform_int_distribution<> dist(0, 2);
    int choice = dist(gen);

    common::types::MappingStepCommand command;
    command.scan_orientation = state.heading;

    if (choice == 0 || choice == 1) { // 66% chance advance
        command.movement = common::types::MovementCommand{
            common::types::MovementCommandType::Advance,
            common::types::RotationDirection::Right,
            common::HorizontalAngle(0 * common::deg),
            drone_config_.max_advance
        };
    } else { // 33% chance rotate
        command.movement = common::types::MovementCommand{
            common::types::MovementCommandType::Rotate,
            common::types::RotationDirection::Right,
            common::HorizontalAngle(90 * common::deg),
            common::PhysicalLength(0 * common::cm)
        };
    }

    return command;
}

REGISTER_MAPPING_ALGORITHM(SlowRandomAlgorithmImpl);

} // namespace algorithm_330371063_324976703
