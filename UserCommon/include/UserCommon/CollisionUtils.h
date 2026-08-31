#pragma once

#include <Common/IMap3D.h>
#include <Common/Types.h>
#include <algorithm>

namespace user_common_330371063_324976703 {
using namespace common;

class CollisionUtils {
public:
    /**
     * @brief Checks if a drone at the given position with the given radius 
     *        collides with any occupied voxels in the map.
     * 
     * @param map The map to check against
     * @param pos The center position of the drone
     * @param radius The collision radius of the drone
     * @return true if collision detected (intersects with Occupied voxel), false otherwise
     */
    static bool isDroneColliding(const IMap3D& map, const Position3D& pos, PhysicalLength radius);

    /**
     * @brief Checks if a drone's entire bounding box is within the map's boundaries.
     * 
     * @param map The map to check against
     * @param pos The center position of the drone
     * @param radius The collision radius of the drone
     * @return true if the drone is fully inside the map boundaries, false otherwise
     */
    static bool isDroneFullyInBounds(const IMap3D& map, const Position3D& pos, PhysicalLength radius);

    /**
     * @brief Iterates over every sample point inside a sphere defined by the drone's position and radius.
     *        The step size is calculated based on the given resolution.
     * 
     * @param center The center position of the sphere
     * @param radius The radius of the sphere
     * @param resolution The resolution of the map (used to compute step size)
     * @param callback A function taking `const Position3D&` and returning a `bool`.
     *                 Return `false` from the callback to abort iteration early, `true` to continue.
     */
    template <typename Func>
    static void forEachVoxelInDroneSphere(const Position3D& center, 
                                          PhysicalLength radius, 
                                          PhysicalLength resolution, 
                                          Func callback) {
        double drone_r_cm = radius.numerical_value_in(cm);
        double res = resolution.numerical_value_in(cm);
        double step = std::max(1.0, res / 2.0);

        for (double dx = -drone_r_cm; dx <= drone_r_cm; dx += step) {
            for (double dy = -drone_r_cm; dy <= drone_r_cm; dy += step) {
                for (double dz = -drone_r_cm; dz <= drone_r_cm; dz += step) {
                    if ((dx * dx + dy * dy + dz * dz) > (drone_r_cm * drone_r_cm)) continue;
                    Position3D p{
                        center.x + XLength(dx * x_extent[cm]),
                        center.y + YLength(dy * y_extent[cm]),
                        center.z + ZLength(dz * z_extent[cm])
                    };
                    
                    if (!callback(p)) {
                        return;
                    }
                }
            }
        }
    }
};

} // namespace user_common_330371063_324976703
