#include <Common/MappingAlgorithmRegistration.h>
#include "MappingAlgorithmRegistrar.h"

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory factory) {
    ::simulator::MappingAlgorithmRegistrar::getInstance().registerFactory(std::move(factory));
}

} // namespace common
