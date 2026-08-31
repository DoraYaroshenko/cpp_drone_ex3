#include <UserCommon/CollisionUtils.h>

#include <algorithm>


namespace user_common_330371063_324976703 {
using namespace common;



bool CollisionUtils::isDroneColliding(const IMap3D& map, const Position3D& pos, PhysicalLength radius) {
    // Quick check to make sure the BB center itself isn't occupied
    if (map.isInBounds(pos) && map.atVoxel(pos) == common::types::VoxelOccupancy::Occupied) {
        return true;
    }
    
    // We get the map resolution by retrieving its config.
    auto cfg = map.getMapConfig();
    PhysicalLength res = cfg.resolution;
    
    bool collided = false;
    forEachVoxelInDroneSphere(pos, radius, res, [&](const Position3D& p) {
        if (map.isInBounds(p) && map.atVoxel(p) == common::types::VoxelOccupancy::Occupied) {
            collided = true;
            return false; // Stop iteration
        }
        return true; // Continue iteration
    });
    
    return collided;
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

    //evaluated at compilation time
    constexpr double EPSILON = 1e-4;

    bool valid = (px - r_val >= min_x - EPSILON && px + r_val <= max_x + EPSILON &&
                  py - r_val >= min_y - EPSILON && py + r_val <= max_y + EPSILON &&
                  pz - r_val >= min_z - EPSILON && pz + r_val <= max_z + EPSILON);

    return valid;
}



} // namespace user_common_330371063_324976703
