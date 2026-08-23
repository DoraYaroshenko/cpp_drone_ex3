#include <UserCommon/CollisionUtils.h>

#include <algorithm>


namespace user_common_330371063_324976703 {
using namespace common;



bool CollisionUtils::isDroneColliding(const IMap3D& map, const Position3D& pos, PhysicalLength radius) {
    // Quick check to make sure the BB center itself isn't occupied
    if (map.isInBounds(pos) && map.atVoxel(pos) == common::types::VoxelOccupancy::Occupied) {
        return true;
    }

    double drone_r_cm = radius.numerical_value_in(cm);
    
    // We get the map resolution by retrieving its config.
    // Wait, IMap3D doesn't necessarily expose resolution, it exposes getMapConfig().
    // Yes, Map3DImpl implements getMapConfig(), and IMap3D exposes it.
    auto cfg = map.getMapConfig();
    double res = cfg.resolution.numerical_value_in(cm);
    
    // Step through the bounding box
    double step = std::max(1.0, res / 2.0); 
    
    for (double dx = -drone_r_cm; dx <= drone_r_cm; dx += step) {
        for (double dy = -drone_r_cm; dy <= drone_r_cm; dy += step) {
            for (double dz = -drone_r_cm; dz <= drone_r_cm; dz += step) {
                // Spherical check for more accurate collision
                if ((dx*dx + dy*dy + dz*dz) > (drone_r_cm*drone_r_cm)) {
                    continue; // outside the spherical radius
                }

                Position3D p{
                    pos.x + XLength(dx * x_extent[cm]),
                    pos.y + YLength(dy * y_extent[cm]),
                    pos.z + ZLength(dz * z_extent[cm])
                };
                
                if (map.isInBounds(p) && map.atVoxel(p) == common::types::VoxelOccupancy::Occupied) {
                    return true;
                }
            }
        }
    }
    
    return false;
}

bool CollisionUtils::isDroneFullyInBounds(const IMap3D& map, const Position3D& pos, PhysicalLength radius) {
    auto bounds = map.getMapConfig().boundaries;
    
    double px = pos.x.numerical_value_in(cm);
    double py = pos.y.numerical_value_in(cm);
    double pz = pos.z.numerical_value_in(cm);
    double r_val = radius.numerical_value_in(cm);
    
    double min_x = bounds.min_x.numerical_value_in(cm);
    double max_x = bounds.max_x.numerical_value_in(cm);
    double min_y = bounds.min_y.numerical_value_in(cm);
    double max_y = bounds.max_y.numerical_value_in(cm);
    double min_z = bounds.min_height.numerical_value_in(cm);
    double max_z = bounds.max_height.numerical_value_in(cm);

    constexpr double EPSILON = 1e-4;

    bool valid = (px - r_val >= min_x - EPSILON && px + r_val <= max_x + EPSILON &&
                  py - r_val >= min_y - EPSILON && py + r_val <= max_y + EPSILON &&
                  pz - r_val >= min_z - EPSILON && pz + r_val <= max_z + EPSILON);

    return valid;
}



} // namespace user_common_330371063_324976703
