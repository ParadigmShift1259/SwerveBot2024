
#pragma once

#include "Constants.h"
#include "ConstantsDigitalOut.h"
#include "ConstantsCANIDs.h"


#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include <rev/CANSparkMax.h>

using namespace ctre::phoenix::motorcontrol::can;

constexpr double kIngestSpeed = 1.0;
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

    // Retracts the intake into the robot
    void RetractIntake();

private:
    /// 775 that runs intake
    TalonSRX m_motor;
    frc::Timer m_timer;

#ifdef OVERUNDER
    rev::CANSparkMax m_deployMotor;
    
    rev::SparkPIDController m_deployMotorPIDController = m_deployMotor.GetPIDController();
#endif

    static constexpr bool kIntakeExtend = true;
    static constexpr bool kIntakeRetract = false;
};