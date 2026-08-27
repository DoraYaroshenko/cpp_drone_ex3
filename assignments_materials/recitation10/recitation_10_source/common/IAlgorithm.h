#pragma once

namespace Recitation9 {

struct Range {
    int minimum = 0;
    int maximum = 0;
};

class IAlgorithm {
public:
    virtual ~IAlgorithm() = default;
    [[nodiscard]] virtual int getNext() = 0;
};

} // namespace Recitation9
