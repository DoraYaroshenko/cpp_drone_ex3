#pragma once

#include <Common/IMap3D.h>

namespace common {

struct MappingAlgorithmDependencies {
    const types::MissionConfigData& mission_config;
    const types::LidarConfigData& lidar_config;
    const types::DroneConfigData& drone_config;
    const IMap3D& output_map; // Algorithms may inspect the map but must not edit it directly. Mission Control edits the map
};

class IMappingAlgorithm {
public:
    virtual ~IMappingAlgorithm() = default;

    explicit IMappingAlgorithm(MappingAlgorithmDependencies dependencies)
        : mission_config_(dependencies.mission_config),
          lidar_config_(dependencies.lidar_config),
          drone_config_(dependencies.drone_config),
          output_map_(dependencies.output_map) {}

    [[nodiscard]] virtual types::MappingStepCommand nextStep(
        const types::DroneState& state,
        const types::LidarScanResult* latest_scan) = 0;

    //just virtual - requires default implementation, optional override, can be instantiated directly
    //virtual and =0 - pure virtual, forces subclass to implement it, cannot be instantiated directly

protected: //If the base class declares a variable as private, it is telling the C++ compiler: "I don't care who inherits from me. The ONLY code that is allowed to read or write this variable is the code physically typed inside my own curly braces { }
    const types::MissionConfigData mission_config_;
    const types::LidarConfigData lidar_config_;
    const types::DroneConfigData drone_config_;
    const IMap3D& output_map_;
};

//don't store the struct instead of many data members, because the struct contains references

} // namespace common
