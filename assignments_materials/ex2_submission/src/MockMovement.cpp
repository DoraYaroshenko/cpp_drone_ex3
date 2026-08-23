#include <drone_mapper/MockMovement.h>

#include <mp-units/systems/si/math.h>

#include <cmath>

namespace drone_mapper {

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    // Left = positive (counter-clockwise), Right = negative (clockwise).
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    // Port from ex1's MockMovementDriver::advance.
    // Moves the drone in the direction it is currently facing (horizontal plane).
    const Position3D position = gps_.position();
    const Orientation orientation = gps_.heading();

    auto dx = si::cos(orientation.horizontal);
    auto dy = si::sin(orientation.horizontal);

    XLength new_x = position.x + XLength((distance * dx).numerical_value_in(cm) * x_extent[cm]);

    YLength new_y = position.y + YLength((distance * dy).numerical_value_in(cm) * y_extent[cm]);

    gps_.setPosition(Position3D{new_x, new_y, position.z});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    // Port from ex1's MockMovementDriver::elevate.
    // Moves the drone vertically (z axis). Negative distance = descend.
    const Position3D position = gps_.position();
    ZLength new_z = position.z + ZLength(distance);
    gps_.setPosition(Position3D{position.x, position.y, new_z});
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper
