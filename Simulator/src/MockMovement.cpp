#include <Simulator/MockMovement.h>

#include <mp-units/systems/si/math.h>

#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <UserCommon/CollisionUtils.h>

namespace {
using namespace common;

void checkMoveSafety(const IMap3D& map, const Position3D& pos, PhysicalLength radius) {
    if (!user_common_330371063_324976703::CollisionUtils::isDroneFullyInBounds(map, pos, radius)) {
        throw std::runtime_error("Collision detected: Drone went out of bounds");
    }
    
    PhysicalLength res = map.getMapConfig().resolution;
    PhysicalLength step = (res / 2.0 < PhysicalLength(1.0 * cm)) ? PhysicalLength(1.0 * cm) : (res / 2.0);

    auto r_sq = radius * radius;

    for (PhysicalLength dx = -radius; dx <= radius; dx += step) {
        for (PhysicalLength dy = -radius; dy <= radius; dy += step) {
            for (PhysicalLength dz = -radius; dz <= radius; dz += step) {
                if ((dx*dx + dy*dy + dz*dz) > r_sq) continue;

                // Explicitly converting to specific extent lengths is necessary here
                Position3D p{
                    pos.x + XLength(dx),
                    pos.y + YLength(dy),
                    pos.z + ZLength(dz)
                };
                
                if (map.isInBounds(p)) {
                    auto v = map.atVoxel(p);
                    if (v == common::types::VoxelOccupancy::Occupied || 
                        v == common::types::VoxelOccupancy::PotentiallyOccupied) {
                        throw std::runtime_error("Collision detected: Drone hit a wall");
                    }
                }
            }
        }
    }
}

void checkPathSafety(const IMap3D& map, const Position3D& start, const Position3D& end, PhysicalLength radius) {
    auto dx = end.x - start.x;
    auto dy = end.y - start.y;
    auto dz = end.z - start.z;
    
    // We must extract to double here because dx, dy, and dz are orthogonal extents 
    // (XLength, YLength, ZLength). mp-units strictly forbids adding x_extent^2 to y_extent^2.
    double dx_val = dx.numerical_value_in(cm);
    double dy_val = dy.numerical_value_in(cm);
    double dz_val = dz.numerical_value_in(cm);
    PhysicalLength total_dist(std::sqrt(dx_val*dx_val + dy_val*dy_val + dz_val*dz_val) * cm);
    
    PhysicalLength res = map.getMapConfig().resolution;
    PhysicalLength step = (res / 2.0 < 1.0 * cm) ? PhysicalLength(1.0 * cm) : (res / 2.0);
    
    for (PhysicalLength d = 0.0 * cm; d < total_dist; d += step) {
        // Native dimensionless quantity handles multiplication natively!
        auto t = d / total_dist;
        
        Position3D inter_pos{
            start.x + dx * t,
            start.y + dy * t,
            start.z + dz * t
        };
        checkMoveSafety(map, inter_pos, radius);
    }
    
    checkMoveSafety(map, end, radius);
}

} // namespace




namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;
namespace user_common_330371063_324976703 {}
using namespace user_common_330371063_324976703;

MockMovement::MockMovement(MockGPS& gps, const common::IMap3D& hidden_map, common::PhysicalLength drone_radius) 
    : gps_(gps), hidden_map_(hidden_map), drone_radius_(drone_radius) {}

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
    Position3D new_pos{new_x, new_y, position.z};

    checkPathSafety(hidden_map_, position, new_pos, drone_radius_);
    gps_.setPosition(new_pos);
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    // Port from ex1's MockMovementDriver::elevate.
    // Moves the drone vertically (z axis). Negative distance = descend.
    const Position3D position = gps_.position();
    ZLength new_z = position.z + ZLength(distance);
    Position3D new_pos{position.x, position.y, new_z};

    checkPathSafety(hidden_map_, position, new_pos, drone_radius_);
    gps_.setPosition(new_pos);
    return types::MovementResult{true, {}};
}




} // namespace simulator
