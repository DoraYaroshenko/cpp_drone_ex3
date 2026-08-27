#pragma once

#include <Common/IMappingAlgorithm.h>
#include <Common/types/DroneTypes.h>

namespace algorithm_330371063_324976703 {

class SimpleSweepAlgorithmImpl : public common::IMappingAlgorithm {
public:
    explicit SimpleSweepAlgorithmImpl(common::MappingAlgorithmDependencies dependencies);

    ~SimpleSweepAlgorithmImpl() override = default;

    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

private:
    bool rotate_next_{false};
};

} // namespace algorithm_330371063_324976703
