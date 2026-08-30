#pragma once

#include <Common/IMappingAlgorithm.h>

#include <functional>
#include <memory>

namespace common {

using MappingAlgorithmFactory =
    std::function<std::unique_ptr<IMappingAlgorithm>(MappingAlgorithmDependencies)>;
//std::function - acts as a container that can hold any callable block of code, as long as its signature matches what's inside the angled brackets
//std::unique_ptr<IMappingAlgorithm> - return type of the function
//(MappingAlgorithmDependencies) - parameter of the function
} // namespace common
