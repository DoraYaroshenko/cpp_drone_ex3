#pragma once

#include <Common/IMap3D.h>
#include <Common/Types.h>


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
};



} // namespace user_common_330371063_324976703
