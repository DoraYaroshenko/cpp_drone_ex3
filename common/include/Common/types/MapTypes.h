#pragma once

#include <Common/Units.h>

namespace common::types {

enum class VoxelOccupancy : signed char {
    PotentiallyOccupied = -3, //obstacle is too close to accurately measure
    OutOfBounds = -2,
    Unmapped = -1,
    Empty = 0,
    Occupied = 1,
};

struct MappingBounds {
    XLength min_x{};
    XLength max_x{};
    YLength min_y{};
    YLength max_y{};
    ZLength min_height{};
    ZLength max_height{};
};

struct MappedVoxel {
    Position3D position{};
    VoxelOccupancy value = VoxelOccupancy::Unmapped;
};

// offset translates mission-relative coordinates into this map's local frame.
struct MapConfig {
    MappingBounds boundaries{};
    Position3D offset{};
    PhysicalLength resolution{}; //length of each voxel edge
};

} // namespace common::types
