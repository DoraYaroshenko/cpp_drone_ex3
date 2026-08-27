#pragma once

#include <common/IAlgorithm.h>

namespace Algorithm2 {

class CyclingAlgorithm final : public Recitation9::IAlgorithm {
public:
    explicit CyclingAlgorithm(Recitation9::Range range);
    [[nodiscard]] int getNext() override;

private:
    Recitation9::Range range_;
    int next_;
};

} // namespace Algorithm2
