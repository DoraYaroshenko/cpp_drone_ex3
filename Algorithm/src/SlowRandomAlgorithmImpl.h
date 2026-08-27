#pragma once

#include <Common/IMappingAlgorithm.h>
#include <Common/types/DroneTypes.h>

namespace algorithm_330371063_324976703 {

class SlowRandomAlgorithmImpl : public common::IMappingAlgorithm {
public:
    explicit SlowRandomAlgorithmImpl(common::MappingAlgorithmDependencies dependencies);

    ~SlowRandomAlgorithmImpl() override = default;

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;
};

} // namespace algorithm_330371063_324976703
