#pragma once

#include <common/AlgorithmFactory.h>

#include <memory>
#include <string>
#include <utility>

namespace Recitation9 {

struct AlgorithmRegistration {
    AlgorithmRegistration(std::string name, AlgorithmFactory factory);
};

} // namespace Recitation9

#define REGISTER_ALGORITHM(class_name) \
    [[maybe_unused]] ::Recitation9::AlgorithmRegistration register_me_##class_name{ \
        #class_name, \
        [](::Recitation9::Range range) -> std::unique_ptr<::Recitation9::IAlgorithm> { \
            return std::make_unique<class_name>(std::move(range)); \
        }}
