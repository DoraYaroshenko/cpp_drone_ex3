#pragma once

#include <Common/Types.h>

namespace common {

class IGPS {
public:
    virtual ~IGPS() = default;
    [[nodiscard]] virtual Position3D position() const = 0; //const means the function will not modify the object itself
    [[nodiscard]] virtual Orientation heading() const = 0;
};

} // namespace common
