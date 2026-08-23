#include <drone_mapper/MapsComparison.h>

#include <cmath>
#include <iostream>

namespace drone_mapper {

std::vector<double> MapsComparison::compare(const IMap3D& original,
                                             const std::vector<IMap3D*> targets) {
    std::vector<double> scores;

    const types::MapConfig orig_config = original.getMapConfig();
    const auto& bounds = orig_config.boundaries;
    PhysicalLength res = orig_config.resolution;

    if (res <= 0.0 * cm) {
        // Invalid resolution — return -1 for all targets.
        for (std::size_t i = 0; i < targets.size(); ++i) {
            scores.push_back(-1.0);
        }
        return scores;
    }

    // Determine iteration bounds from the original map's boundaries.
    ZLength min_z = bounds.min_height;
    ZLength max_z = bounds.max_height;
    YLength min_y = bounds.min_y;
    YLength max_y = bounds.max_y;
    XLength min_x = bounds.min_x;
    XLength max_x = bounds.max_x;

    // Convert the general PhysicalLength resolution into axis-specific lengths once.
    // (This is necessary because x_extent, y_extent, and z_extent are different quantity specs)
    double res_cm = res.numerical_value_in(cm);
    ZLength res_z(res_cm * z_extent[cm]);
    YLength res_y(res_cm * y_extent[cm]);
    XLength res_x(res_cm * x_extent[cm]);

    for (const IMap3D* target : targets) {
        if (target == nullptr) {
            scores.push_back(-1.0);
            continue;
        }

        double score = 0.0;
        double max_score = 0.0;

        // Iterate over all voxels in the original map's bounds using strongly-typed units.
        for (ZLength z = min_z + res_z * 0.5; z < max_z; z += res_z) {
            for (YLength y = min_y + res_y * 0.5; y < max_y; y += res_y) {
                for (XLength x = min_x + res_x * 0.5; x < max_x; x += res_x) {
                    Position3D pos{x, y, z};

                    types::VoxelOccupancy orig_val = original.atVoxel(pos);
                    types::VoxelOccupancy target_val = target->atVoxel(pos);

                    // Skip out-of-bounds voxels in the target.
                    if (target_val == types::VoxelOccupancy::OutOfBounds) {
                        continue;
                    }

                    max_score += 1.0;

                    if (orig_val == target_val) {
                        // Exact match.
                        score += 1.0;
                    } else if (target_val == types::VoxelOccupancy::Unmapped) {
                        // Unmapped (-1) in the target = unreachable.
                        // Partial credit (same as ex1's -1 unreachable penalty).
                        score += 0.25;
                    } else if (target_val == types::VoxelOccupancy::PotentiallyOccupied) {
                        // PotentiallyOccupied (-3) = lidar sensed something too close.
                        // Give partial credit — better than a complete mismatch.
                        score += 0.25;
                    }
                    // else: occupied/free mismatch → score += 0.0
                }
            }
        }

        if (max_score == 0.0) {
            scores.push_back(100.0); // No comparable voxels → perfect score.
        } else {
            scores.push_back((score / max_score) * 100.0); // Scale to 0-100.
        }
    }

    return scores;
}

} // namespace drone_mapper
