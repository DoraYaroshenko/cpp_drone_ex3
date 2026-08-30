#pragma once

#include <Common/Units.h>

#include <cstddef>
#include <vector>

namespace common::types {

struct LidarConfigData {
    PhysicalLength z_min{}; //minimal operational distance from the lidar
    PhysicalLength z_max{};
    PhysicalLength d{}; //spacing between circles
    std::size_t fov_circles = 0; //number of beam circles. number of beams on each circle grows exponentially with 4
};

struct LidarHit { //when a specific beam hits an obstacle
    PhysicalLength distance{};
    Orientation angle{};
};

using LidarScanResult = std::vector<LidarHit>; //Whenever I write LidarScanResult in the code, I actually mean std::vector<LidarHit>

} // namespace common::types
