
#pragma once

#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

#include "ConstantsDigitalOut.h"
#include "ConstantsCANIDs.h"

constexpr double kIngestSpeed = 0.8;
constexpr double kReleaseSpeed = -0.8;

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
    // TalonSRX m_motor;
    frc::Timer m_timer;

    static constexpr bool kIntakeExtend = true;
    static constexpr bool kIntakeRetract = false;
};