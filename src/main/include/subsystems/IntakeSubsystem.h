#pragma once

#include "Constants.h"
#include "ConstantsDigitalInputs.h"
#include "ConstantsCANIDs.h"

#include <frc/DigitalInput.h>

#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/WaitCommand.h>

#include <wpi/deprecated.h>
WPI_IGNORE_DEPRECATED
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
WPI_UNIGNORE_DEPRECATED

#include <rev/CANSparkMax.h>

using namespace ctre::phoenix::motorcontrol::can;

constexpr double kIngestSpeed = 0.4;
constexpr double kReleaseSpeed = -1.0;

class IntakeSubsystem : public frc2::SubsystemBase
{
public:
    IntakeSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Drives the intake at a given speed
    /// \param speed         Desired motor speed to run, ranging from [-1, 1]
    void Set(double speed);
    /// Extends the intake out of the robot
    void ExtendIntake();
    /// Adjusts the intake to allow for higher shooting angles
    void AdjustIntake();
    // Retracts the intake into the robot
    void RetractIntake();
    bool IsNotePresent() { frc2::WaitCommand(0.25_s); return m_photoEye.Get(); }
    void EjectNote() { Set(kIngestSpeed); }
    void Stop() { Set(0.0); }

private:
    /// 775 that runs intake
    TalonSRX m_motor;
    frc::Timer m_timer;
    frc::DigitalInput m_photoEye;

#ifdef OVERUNDER
    rev::CANSparkMax m_deployMotor;
    rev::SparkRelativeEncoder m_deployRelativeEnc = m_deployMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);    
    rev::SparkPIDController m_deployPIDController = m_deployMotor.GetPIDController();
#endif

    static constexpr bool kIntakeExtend = true;
    static constexpr bool kIntakeRetract = false;
};
