#pragma once
#include "Constants.h"

constexpr int kFrontLeftDriveCANID    = 1;       //!< Front Left Drive CAN ID (TalonFX)   
constexpr int kFrontLeftTurningCANID  = 2;       //!< Front Left Turn CAN ID (SparkMAX)   

constexpr int kFrontRightDriveCANID   = 3;       //!< Front Right Drive CAN ID (TalonFX)   
constexpr int kFrontRightTurningCANID = 4;       //!< Front Right Turn CAN ID (SparkMAX)

constexpr int kRearRightDriveCANID    = 5;       //!< Rear Right Drive CAN ID (TalonFX)   
constexpr int kRearRightTurningCANID  = 6;       //!< Rear Right Turn CAN ID (SparkMAX)

constexpr int kRearLeftDriveCANID     = 7;       //!< Rear Left Drive CAN ID (TalonFX)   
constexpr int kRearLeftTurningCANID   = 8;       //!< Rear Left Turn CAN ID (SparkMAX)


constexpr int kIntakeRollerCANID     = 11;       // TalonSRX
#ifdef OVERUNDER
constexpr int kIntakeDeployCANID      = 15;       // SparkMax
#else
constexpr int kIntakeDeployCANID      = 14;       // SparkMax
constexpr int kIntakeDeployFollowCANID = 16;       // SparkMax
#endif

constexpr int kShooterOverWheelsCANID  = 13;
constexpr int kShooterUnderWheelsCANID = 12;

#ifdef OVERUNDER
constexpr int kShooterBackWheelsCANID = 22;
constexpr int kShooterElevationControllerCANID = 23;
constexpr int kElevationEncoderCANID = 1;
#else
constexpr int kShooterElevationControllerCANID = 15;

constexpr int kPlopperElevationControllerCANID = 23; // SparkMax

constexpr int kPlopperRollerCANID     = 12;       // TalonSRX
#endif