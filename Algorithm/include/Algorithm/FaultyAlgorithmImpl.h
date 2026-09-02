#pragma once

#include <Common/IMappingAlgorithm.h>

namespace algorithm_330371063_324976703 {

class FaultyAlgorithmImpl final : public common::IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;
};

} // namespace algorithm_330371063_324976703
