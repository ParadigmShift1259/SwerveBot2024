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

using namespace ctre::phoenix::motorcontrol::can;

class AmpSubsystem : public frc2::SubsystemBase
{
  public:
    AmpSubsystem();
    void Periodic();

    void GoToPosition(double position);

    void AmpPosition() { GoToPosition(m_AmpTurns); }
    void ParkPosition() { GoToPosition(m_ParkTurns); }
    void TransferPosition() { GoToPosition(m_TransferTurns); }
    void TrapPosition() { GoToPosition(m_TrapTurns); }

    /// Drives the plopper motor at a given speed
    /// \param speed         Desired motor speed to run, ranging from [-1, 1]
    void Set(double speed);
    void Stop() { Set(0.0); }
    
  private:
    frc::Timer m_timer;
    // frc::DigitalInput m_photoEye;

    TalonSRX m_motor;

    rev::CANSparkMax m_liftMotor;
    rev::SparkRelativeEncoder m_liftRelativeEnc = m_liftMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);    
    rev::SparkPIDController m_liftPIDController = m_liftMotor.GetPIDController();

    double m_AmpTurns;    
    double m_ParkTurns;   
    double m_TransferTurns;
    double m_TrapTurns;   

    double m_plopperPosition;
    double m_plopperSpeed;

    wpi::log::DoubleLogEntry m_log;
};