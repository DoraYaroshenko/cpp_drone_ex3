#pragma once

#include <common/IAlgorithm.h>

#include <functional>
#include <memory>

namespace Recitation9 {

using AlgorithmFactory = std::function<std::unique_ptr<IAlgorithm>(Range)>;

} // namespace Recitation9
