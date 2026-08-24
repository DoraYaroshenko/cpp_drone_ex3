#include <Common/MissionControlRegistration.h>
#include "MissionControlRegistrar.h"

namespace common {

MissionControlRegistration::MissionControlRegistration(MissionControlFactory factory) {
    ::simulator::MissionControlRegistrar::getInstance().registerFactory(std::move(factory));
}

} // namespace common
