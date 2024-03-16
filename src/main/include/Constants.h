#pragma once

#include <units/angle.h>

//#define THING1

constexpr double kMinOut = -1.0;
constexpr double kMaxOut = 1.0;

constexpr double kDeployMinOut = -0.5;
constexpr double kDeployMaxOut = 0.5;

constexpr double kIntakeDeployGearRatio = 22.0 / 36.0;  // 22T sprocket input, 36T sprocket output

constexpr double c_defaultRetractTurns = 4.0;
constexpr double c_defaultExtendTurns = 51.0;
constexpr double c_defaultOffsetTurns = -5.0;

constexpr units::angle::degree_t c_defaultTravelPosition = 33.0_deg;
constexpr units::angle::degree_t c_defaultStartPosition = 66.0_deg;