#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Types.h>


namespace user_common_330371063_324976703 {
using namespace common;



class ScanResultToVoxels {
public:
    // Applies a LiDAR scan directly to the output map.
    //
    // The converter writes only scan observation states:
    // Occupied, Empty, and PotentiallyOccupied (A new state).
    static void applyToMap(IMutableMap3D& output_map,
                           const Position3D& scan_origin,
                           const Orientation& drone_heading,
                           const common::types::LidarScanResult& scan,
                           const common::types::LidarConfigData& lidar_config);
                           
};



} // namespace user_common_330371063_324976703
