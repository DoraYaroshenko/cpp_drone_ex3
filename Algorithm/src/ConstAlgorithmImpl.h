#pragma once

#include <Common/IMappingAlgorithm.h>
#include <Common/types/DroneTypes.h>

namespace algorithm_330371063_324976703 {

class ConstAlgorithmImpl : public common::IMappingAlgorithm {
public:
    explicit ConstAlgorithmImpl(common::MappingAlgorithmDependencies dependencies);

    ~ConstAlgorithmImpl() override = default;

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;
};

} // namespace algorithm_330371063_324976703
