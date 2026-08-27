#include <algorithm2/CyclingAlgorithm.h>

#include <common/AlgorithmRegistration.h>

namespace Algorithm2 {

CyclingAlgorithm::CyclingAlgorithm(Recitation9::Range range)
    : range_(range), next_(range.minimum) {}

int CyclingAlgorithm::getNext() {
    const int result = next_;
    next_ = next_ == range_.maximum ? range_.minimum : next_ + 1;
    return result;
}

REGISTER_ALGORITHM(CyclingAlgorithm);

} // namespace Algorithm2
