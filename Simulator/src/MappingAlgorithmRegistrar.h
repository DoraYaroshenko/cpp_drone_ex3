#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <vector>
#include <string>

namespace simulator {

class MappingAlgorithmRegistrar {
public:
    static MappingAlgorithmRegistrar& getInstance() {
        static MappingAlgorithmRegistrar instance;
        return instance;
    }

    void registerFactory(common::MappingAlgorithmFactory factory) {
        factories_.push_back(std::move(factory));
    }

    const std::vector<common::MappingAlgorithmFactory>& getFactories() const {
        return factories_;
    }

    void clear() {
        factories_.clear();
    }

private:
    MappingAlgorithmRegistrar() = default;
    ~MappingAlgorithmRegistrar() = default;
    MappingAlgorithmRegistrar(const MappingAlgorithmRegistrar&) = delete;
    MappingAlgorithmRegistrar& operator=(const MappingAlgorithmRegistrar&) = delete;

    std::vector<common::MappingAlgorithmFactory> factories_;
};

} // namespace simulator
