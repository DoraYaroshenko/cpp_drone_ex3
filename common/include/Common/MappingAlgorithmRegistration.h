#pragma once

#include <Common/MappingAlgorithmFactory.h>

#include <utility> //gives std::move

namespace common {

struct MappingAlgorithmRegistration {
    explicit MappingAlgorithmRegistration(MappingAlgorithmFactory factory);
};

} // namespace common

//Whenever you see REGISTER_MAPPING_ALGORITHM(Something), replace it with all the code below, and substitute Something wherever you see class_name
//[[maybe_unused]] ::common::MappingAlgorithmRegistration register_me_##class_name - global variable
//[](::common::MappingAlgorithmDependencies dependencies)c-> std::unique_ptr<::common::IMappingAlgorithm> - the lambda function which is a factory that the constructor expects
//std::make_unique<class_name>(std::move(dependencies)) - calls the constructor of classname

#define REGISTER_MAPPING_ALGORITHM(class_name)                                      \
    [[maybe_unused]] ::common::MappingAlgorithmRegistration register_me_##class_name{ \
        [](::common::MappingAlgorithmDependencies dependencies)                     \
            -> std::unique_ptr<::common::IMappingAlgorithm> {                       \
            return std::make_unique<class_name>(std::move(dependencies));            \
        }}

//maybe_unused - don't throw a warning if the variable is not used
