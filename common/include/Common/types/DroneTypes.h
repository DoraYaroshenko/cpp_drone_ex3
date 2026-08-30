#pragma once

#include <Common/Units.h>

#include <cstddef> //provides size_t, NULL, std::byte
#include <optional> //template class that allows optional contained value, meaning it either contains value or std::nullopt
#include <string>

namespace common::types {

struct DroneConfigData {
    PhysicalLength radius{};
    HorizontalAngle max_rotate{};
    PhysicalLength max_advance{};
    PhysicalLength max_elevate{};
};

enum class RotationDirection { Left, Right }; //fixed set of named constants
enum class MovementCommandType { Hover, Rotate, Advance, Elevate }; //hover - do nothing

struct MovementCommand {
    MovementCommandType type = MovementCommandType::Hover;
    RotationDirection rotation = RotationDirection::Left;
    HorizontalAngle angle{}; //{} - initialization to default value 
    PhysicalLength distance{};
};

enum class AlgorithmStatus { Working, Finished, FinishedWithUnmappableVoxels };

struct MappingStepCommand {
    std::optional<MovementCommand> movement{}; //optional, because at each step the algorithm can choose to move, scan, do both or neither 
    std::optional<Orientation> scan_orientation{};
    AlgorithmStatus status = AlgorithmStatus::Working;
};

struct MovementResult {
    bool success = true;
    std::string message{};

    [[nodiscard]] explicit operator bool() const noexcept { return success; } //nodiscard - doesn't allow to throw result, explicit - prevents automatic conversion to boolean, operator bool() - custom conversion, const - we will not modify the object, noexcept - we will not throw an exception
};

struct DroneState {
    Position3D position{};
    Orientation heading{};
    std::size_t step_index = 0; //steps done by now
};

enum class DroneStepStatus { Continue, Completed, Error };

struct DroneStepResult {
    DroneStepStatus status = DroneStepStatus::Continue;
    std::string message{};
};

} // namespace common::types
