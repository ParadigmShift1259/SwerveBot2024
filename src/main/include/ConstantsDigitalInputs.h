#pragma once

// Swerve absolute encoders on first four digital inputs

constexpr int kElevationLimitRear  = 4;
constexpr int kElevationLimitFront = 5;

#ifdef OVERUNDER  
constexpr int kIntakePhotoeye      = 6;
#else
constexpr int kIntakePhotoeye      = 9;
#endif

