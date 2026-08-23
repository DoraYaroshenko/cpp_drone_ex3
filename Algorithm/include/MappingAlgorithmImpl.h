#pragma once

#include <Common/IMappingAlgorithm.h>

namespace algorithm_330371063_324976703 {
using namespace common;

class MappingAlgorithmImpl_330371063_324976703 final : public IMappingAlgorithm {
public:
    //explicit MappingAlgorithmImpl(const common::types::DroneConfigData drone_config, const IMap3D& output_map);
    using IMappingAlgorithm::IMappingAlgorithm;
    ~MappingAlgorithmImpl_330371063_324976703() override;
    [[nodiscard]] common::types::MappingStepCommand nextStep(const common::types::DroneState& state,
                                                     const common::types::LidarScanResult* latest_scan) override;
};

} // namespace algorithm_330371063_324976703
