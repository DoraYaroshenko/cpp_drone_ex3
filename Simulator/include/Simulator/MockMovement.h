#pragma once

#include <Common/IDroneMovement.h>
#include <Simulator/MockGPS.h>



namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;
namespace user_common_330371063_324976703 {}
using namespace user_common_330371063_324976703;

// Optional implementation for the 
class MockMovement final : public IDroneMovement {
public:
    explicit MockMovement(MockGPS& gps);

    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    types::MovementResult advance(PhysicalLength distance) override;
    types::MovementResult elevate(PhysicalLength distance) override;

private:
    MockGPS& gps_;
};




} // namespace simulator
