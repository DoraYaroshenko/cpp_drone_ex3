#include <drone_mapper/Beam.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_mapper {

namespace {
constexpr double INF = std::numeric_limits<double>::infinity();
// IMPROVEMENT: Tighter epsilon than ex1 (was 1e-12).
// Using 1e-10 provides a larger safety margin for accumulated
// floating-point error in long ray traversals while still being
// far below any meaningful physical distance.
constexpr double BOUNDARY_EPS = 1e-10;
} // namespace

Beam::Beam(Position3D origin, Orientation orient, PhysicalLength resolution)
    : origin_(origin),
      orient_(orient),
      resolution_(resolution),
      dist_traveled_(0.0 * cm) {

    dir_vector_ = orientationToDirection(orient_);

    double res = resolution_.force_numerical_value_in(cm);

    // The distances the beam must travel between successive axis boundaries.
    // IMPROVEMENT: resolution-aware (ex1 hardcoded 1.0 cm per voxel).
    dist_between_axis_boundaries_ = Position3D{
        XLength((dir_vector_.x == 0.0 ? INF : std::abs(res / dir_vector_.x)) * x_extent[cm]),
        YLength((dir_vector_.y == 0.0 ? INF : std::abs(res / dir_vector_.y)) * y_extent[cm]),
        ZLength((dir_vector_.z == 0.0 ? INF : std::abs(res / dir_vector_.z)) * z_extent[cm])
    };

    // The distance from origin to the first axis boundary in each direction.
    dist_to_next_axis_ = Position3D{
        XLength(calculateDistanceToNextBoundaryInAxis(origin_.x, dir_vector_.x, resolution_)),
        YLength(calculateDistanceToNextBoundaryInAxis(origin_.y, dir_vector_.y, resolution_)),
        ZLength(calculateDistanceToNextBoundaryInAxis(origin_.z, dir_vector_.z, resolution_))
    };

    curr_voxel_ = positionToVoxelIndex(origin_, resolution_);

    step_ = VoxelStep{
        (dir_vector_.x > 0) ? 1 : (dir_vector_.x < 0 ? -1 : 0),
        (dir_vector_.y > 0) ? 1 : (dir_vector_.y < 0 ? -1 : 0),
        (dir_vector_.z > 0) ? 1 : (dir_vector_.z < 0 ? -1 : 0)
    };
}

PhysicalLength Beam::getDistanceTraveled() const {
    return dist_traveled_;
}

VoxelIndex Beam::getCurrentVoxel() const {
    return curr_voxel_;
}

void Beam::progressBeamToNextVoxel() {
    PhysicalLength step_dist = std::min({
        dist_to_next_axis_.x.force_numerical_value_in(cm),
        dist_to_next_axis_.y.force_numerical_value_in(cm),
        dist_to_next_axis_.z.force_numerical_value_in(cm)
    }) * cm;

    // Detect tied crossings (edge/corner hits).
    double dx = dist_to_next_axis_.x.force_numerical_value_in(cm);
    double dy = dist_to_next_axis_.y.force_numerical_value_in(cm);
    double dz = dist_to_next_axis_.z.force_numerical_value_in(cm);
    double s = step_dist.force_numerical_value_in(cm);

    bool step_x = std::abs(dx - s) < BOUNDARY_EPS;
    bool step_y = std::abs(dy - s) < BOUNDARY_EPS;
    bool step_z = std::abs(dz - s) < BOUNDARY_EPS;

    dist_traveled_ += step_dist;

    if (step_x) {
        curr_voxel_.x += step_.x;
    }
    if (step_y) {
        curr_voxel_.y += step_.y;
    }
    if (step_z) {
        curr_voxel_.z += step_.z;
    }

    // Update distances to next boundary — reset any axis that was crossed.
    dist_to_next_axis_ = Position3D{
        step_x
            ? dist_between_axis_boundaries_.x
            : dist_to_next_axis_.x - XLength(step_dist),
        step_y
            ? dist_between_axis_boundaries_.y
            : dist_to_next_axis_.y - YLength(step_dist),
        step_z
            ? dist_between_axis_boundaries_.z
            : dist_to_next_axis_.z - ZLength(step_dist)
    };
}

PhysicalLength Beam::calculateDistanceToNextBoundaryInAxis(
        PhysicalLength position_coord, double dir_coord, PhysicalLength res) const {

    double res_cm = res.force_numerical_value_in(cm);

    if (dir_coord > 0.0) {
        // IMPROVEMENT: resolution-aware boundary calculation.
        // Next boundary = ceil(pos / resolution) * resolution, but at least one step forward.
        double pos = position_coord.force_numerical_value_in(cm);
        double voxel_idx = std::floor(pos / res_cm);
        double next_boundary = (voxel_idx + 1.0) * res_cm;
        return (next_boundary * cm - position_coord) / dir_coord;
    } else if (dir_coord < 0.0) {
        double pos = position_coord.force_numerical_value_in(cm);
        double voxel_idx = std::floor(pos / res_cm);
        double current_boundary = voxel_idx * res_cm;
        // IMPROVEMENT: Use a tighter epsilon (was 1e-12 in ex1) for exact-boundary detection.
        double next_boundary =
            (std::fabs(pos - current_boundary) < BOUNDARY_EPS)
                ? current_boundary - res_cm
                : current_boundary;
        return (next_boundary * cm - position_coord) / dir_coord;
    } else {
        return PhysicalLength{INF * cm};
    }
}

} // namespace drone_mapper
