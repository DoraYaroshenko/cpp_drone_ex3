#include <Simulator/SimulationRunFactoryImpl.h>

#include <stdexcept>

namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    return nullptr;
}




} // namespace simulator
