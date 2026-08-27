#pragma once

#include <common/IAlgorithm.h>

#include <random>

namespace Algorithm3 {

class SlowRandomAlgorithm final : public Recitation9::IAlgorithm {
public:
    explicit SlowRandomAlgorithm(Recitation9::Range range);
    [[nodiscard]] int getNext() override;

private:
    std::mt19937 generator_;
    std::uniform_int_distribution<int> distribution_;
};

} // namespace Algorithm3
