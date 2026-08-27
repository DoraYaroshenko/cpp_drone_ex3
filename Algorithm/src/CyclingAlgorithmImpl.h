#pragma once

#include <Common/IMappingAlgorithm.h>
#include <Common/types/DroneTypes.h>

namespace algorithm_330371063_324976703 {

class CyclingAlgorithmImpl : public common::IMappingAlgorithm {
public:
    explicit CyclingAlgorithmImpl(common::MappingAlgorithmDependencies dependencies);

    ~CyclingAlgorithmImpl() override = default;

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

private:
    int step_count_ = 0;
};

} // namespace algorithm_330371063_324976703
