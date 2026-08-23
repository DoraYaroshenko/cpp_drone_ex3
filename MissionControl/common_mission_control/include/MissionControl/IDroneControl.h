#pragma once

#include <Common/Types.h>

namespace mission_control {

using namespace common;

class IDroneControl {
public:
    virtual ~IDroneControl() = default;
    [[nodiscard]] virtual common::types::DroneStepResult step() = 0;
    [[nodiscard]] virtual common::types::DroneState state() const = 0;
};

} // namespace mission_control
