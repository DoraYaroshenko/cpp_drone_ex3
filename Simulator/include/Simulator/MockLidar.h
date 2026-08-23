#pragma once

#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>




namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;
namespace user_common_330371063_324976703 {}
using namespace user_common_330371063_324976703;

class MockLidar final : public ILidar {
public:
    MockLidar(types::LidarConfigData config, const IMap3D& map, const IGPS& gps);

    [[nodiscard]] types::LidarScanResult scan(Orientation scan_orientation) const override;
    // Change: 20.6 - added a config getter
    [[nodiscard]] types::LidarConfigData config() const override;

private:
    [[nodiscard]] PhysicalLength traceBeam(const Orientation& beam) const;

    types::LidarConfigData config_;
    const IMap3D& map_;
    const IGPS& gps_;
};




} // namespace simulator
