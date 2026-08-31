#pragma once

#include <Common/Types.h>

namespace user_common_330371063_324976703 {

struct Direction3D {
    double dx;
    double dy;
    double dz;
};

class OrientationUtils {
public:
    // Calculates a direction vector (dx, dy, dz) from a given orientation.
    static Direction3D getBeamDirection(const common::Orientation& orientation);

    // Calculates the absolute position at a specific distance along a direction vector from an origin point.
    static common::Position3D pointAlongBeam(const common::Position3D& origin, 
                                             const Direction3D& dir, 
                                             common::PhysicalLength distance);

    // Adds two orientations (e.g. drone heading + relative beam orientation).
    static common::Orientation addOrientations(const common::Orientation& a, 
                                               const common::Orientation& b);
};

} // namespace user_common_330371063_324976703
