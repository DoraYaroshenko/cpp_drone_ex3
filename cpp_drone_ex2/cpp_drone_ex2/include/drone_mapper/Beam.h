#pragma once

#include <drone_mapper/Units.h>
#include <mp-units/systems/si/math.h>

#include <cmath>

namespace drone_mapper {

// Voxel index in integer grid coordinates (one voxel per resolution unit).
struct VoxelIndex {
    int x;
    int y;
    int z;

    VoxelIndex operator+(const VoxelIndex& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    VoxelIndex operator-(const VoxelIndex& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    bool operator==(const VoxelIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Step direction for DDA ray marching (one of -1, 0, +1 per axis).
struct VoxelStep {
    int x;
    int y;
    int z;
};

// Direction vector in world space (unit-less).
struct DirectionVector {
    double x;
    double y;
    double z;

    DirectionVector operator*(double mul) const {
        return {x * mul, y * mul, z * mul};
    }
};

[[nodiscard]] inline DirectionVector normalize(DirectionVector v) {
    double mag = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (mag == 0.0) {
        return {0.0, 0.0, 0.0};
    }
    return {v.x / mag, v.y / mag, v.z / mag};
}

// Convert an orientation (horizontal + altitude angles) to a unit direction vector.
[[nodiscard]] inline DirectionVector orientationToDirection(const Orientation& o) {
    double cos_alt = si::cos(o.altitude).numerical_value_in(mp::one);
    return {
        cos_alt * si::cos(o.horizontal).numerical_value_in(mp::one),
        cos_alt * si::sin(o.horizontal).numerical_value_in(mp::one),
        si::sin(o.altitude).numerical_value_in(mp::one)
    };
}

// Convert a unit direction vector back to an orientation.
[[nodiscard]] inline Orientation directionToOrientation(const DirectionVector& d) {
    double horiz = std::sqrt(d.x * d.x + d.y * d.y);
    return {
        HorizontalAngle(si::atan2(d.y * cm, d.x * cm)),
        AltitudeAngle(si::atan2(d.z * cm, horiz * cm))
    };
}

// Convert a world-space Position3D to a voxel index given a resolution.
// Each voxel spans [i*res, (i+1)*res) in each axis.
[[nodiscard]] inline VoxelIndex positionToVoxelIndex(const Position3D& pos, PhysicalLength resolution) {
    double res = resolution.force_numerical_value_in(cm);
    return {
        static_cast<int>(std::floor(pos.x.force_numerical_value_in(cm) / res)),
        static_cast<int>(std::floor(pos.y.force_numerical_value_in(cm) / res)),
        static_cast<int>(std::floor(pos.z.force_numerical_value_in(cm) / res))
    };
}

// Convert a voxel index center back to a world-space Position3D.
[[nodiscard]] inline Position3D voxelIndexToPosition(const VoxelIndex& vi, PhysicalLength resolution) {
    double res = resolution.force_numerical_value_in(cm);
    return {
        XLength((vi.x + 0.5) * res * x_extent[cm]),
        YLength((vi.y + 0.5) * res * y_extent[cm]),
        ZLength((vi.z + 0.5) * res * z_extent[cm])
    };
}

/// DDA-based 3D ray marcher.
///
/// Traces a ray from an origin in a given direction, stepping through voxels
/// one at a time using the DDA (Digital Differential Analyzer) algorithm.
/// This guarantees every intersected voxel is visited exactly once.
///
/// Ported from cpp_drone_ex1's Beam class with the following improvements:
///   - Configurable resolution (no longer assumes 1 voxel = 1 cm).
///   - Uses a tighter epsilon for boundary comparisons to reduce
///     floating-point edge-case misclassifications.
class Beam {
public:
    Beam(Position3D origin, Orientation orient, PhysicalLength resolution);

    [[nodiscard]] PhysicalLength getDistanceTraveled() const;
    [[nodiscard]] VoxelIndex getCurrentVoxel() const;
    void progressBeamToNextVoxel();

private:
    PhysicalLength calculateDistanceToNextBoundaryInAxis(
        PhysicalLength position_coord, double dir_coord, PhysicalLength res) const;

    Position3D origin_;
    Orientation orient_;
    PhysicalLength resolution_;

    PhysicalLength dist_traveled_;
    DirectionVector dir_vector_;

    // Distances expressed in world units (cm), not voxel counts.
    Position3D dist_between_axis_boundaries_;
    Position3D dist_to_next_axis_;

    VoxelIndex curr_voxel_;
    VoxelStep step_;
};

} // namespace drone_mapper
