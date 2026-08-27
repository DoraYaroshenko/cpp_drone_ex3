#include <algorithm3/SlowRandomAlgorithm.h>

#include <common/AlgorithmRegistration.h>

#include <chrono>
#include <thread>

namespace Algorithm3 {

SlowRandomAlgorithm::SlowRandomAlgorithm(Recitation9::Range range)
    : generator_(std::mt19937::default_seed),
      distribution_(range.minimum, range.maximum) {}

int SlowRandomAlgorithm::getNext() {
    // Simulate expensive work without distracting CPU-heavy example code.
    std::this_thread::sleep_for(std::chrono::milliseconds{250});
    return distribution_(generator_);
}

REGISTER_ALGORITHM(SlowRandomAlgorithm);

} // namespace Algorithm3
