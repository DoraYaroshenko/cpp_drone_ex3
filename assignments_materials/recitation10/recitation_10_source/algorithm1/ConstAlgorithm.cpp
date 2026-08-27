#include <algorithm1/ConstAlgorithm.h>

#include <common/AlgorithmRegistration.h>

namespace Algorithm1 {

ConstAlgorithm::ConstAlgorithm(Recitation9::Range range)
    : value_(range.minimum) {}

int ConstAlgorithm::getNext() {
    return value_;
}

REGISTER_ALGORITHM(ConstAlgorithm);

} // namespace Algorithm1
