#pragma once

#include <common/IAlgorithm.h>

namespace Algorithm1 {

class ConstAlgorithm final : public Recitation9::IAlgorithm {
public:
    explicit ConstAlgorithm(Recitation9::Range range);
    [[nodiscard]] int getNext() override;

private:
    int value_;
};

} // namespace Algorithm1
