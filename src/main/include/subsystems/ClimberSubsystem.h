#pragma once

#include <wpi/DataLog.h>

#include "Constants.h"
#include "ConstantsDigitalInputs.h"
#include "ConstantsCANIDs.h"

#include <frc/DigitalInput.h>

#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DataLogManager.h>

#include <wpi/deprecated.h>
WPI_IGNORE_DEPRECATED
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
WPI_UNIGNORE_DEPRECATED

#include <rev/CANSparkMax.h>

class ClimberSubsystem : public frc2::SubsystemBase
{
public:

    ClimberSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Drives the climber at a given speed
    /// \param speed         Desired motor speed to run, ranging from [-1, 1]
    void Set(double speed);

#ifndef THING1
    void Stop() { m_followMotor.StopMotor(); m_leadMotor.StopMotor(); }
#else
    void Stop() { }
#endif

    void GoToPosition(double position);

    void HighPosition() { GoToPosition(m_HighTurns); }
    void ParkPosition() { GoToPosition(m_ParkTurns); }

    enum Position {
        kDefaultPosition,
        kParkPosition = kDefaultPosition,
        kHighPosition
    };

private:
    // 775 that pulls/releases rope for hooks
#ifndef THING1
    rev::CANSparkMax m_leadMotor;
    rev::SparkRelativeEncoder m_leadRelativeEnc = m_leadMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);    
    rev::SparkPIDController m_leadPIDController = m_leadMotor.GetPIDController();
    rev::CANSparkMax m_followMotor;
    rev::SparkRelativeEncoder m_followRelativeEnc = m_followMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);    
    rev::SparkPIDController m_followPIDController = m_followMotor.GetPIDController();
#endif

    double m_HighTurns;    
    double m_ParkTurns;   

    double m_climbLeadPosition = 1.0;
    double m_climbFollowPosition = 1.0;

    wpi::log::DoubleLogEntry m_log;

    double m_leadDirection = 1.0;
    double m_followDirection = 1.0;
};
