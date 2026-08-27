#pragma once

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <MissionControl/IDroneControl.h>
#include <memory>
#include <filesystem>

namespace mission_control_330371063_324976703 {

class SimpleMissionControlImpl : public common::IMissionControl {
public:
    explicit SimpleMissionControlImpl(common::MissionControlDependencies deps);

    ~SimpleMissionControlImpl() override = default;

    [[nodiscard]] common::types::MissionRunResult runMission() override;

private:
    common::types::MissionConfigData mission_;
    common::IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
    std::unique_ptr<mission_control::IDroneControl> drone_control_;
};

} // namespace mission_control_330371063_324976703
