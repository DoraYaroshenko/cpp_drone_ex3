#pragma once

#include <Common/MissionControlFactory.h>
#include <vector>

namespace simulator {

class MissionControlRegistrar {
public:
    static MissionControlRegistrar& getInstance() {
        static MissionControlRegistrar instance;
        return instance;
    }

    void registerFactory(common::MissionControlFactory factory) {
        factories_.push_back(std::move(factory));
    }

    const std::vector<common::MissionControlFactory>& getFactories() const {
        return factories_;
    }

    void clear() {
        factories_.clear();
    }

private:
    MissionControlRegistrar() = default;
    ~MissionControlRegistrar() = default;
    MissionControlRegistrar(const MissionControlRegistrar&) = delete;
    MissionControlRegistrar& operator=(const MissionControlRegistrar&) = delete;

    std::vector<common::MissionControlFactory> factories_;
};

} // namespace simulator
