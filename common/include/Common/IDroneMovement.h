#pragma once

#include <Common/Types.h>

namespace common {

class IDroneMovement {
public:
    virtual ~IDroneMovement() = default; //if you ever delete an object through a pointer to its base class, that base class must have a virtual destructor
    virtual types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) = 0;
    virtual types::MovementResult advance(PhysicalLength distance) = 0;
    virtual types::MovementResult elevate(PhysicalLength distance) = 0;
};

} // namespace common
