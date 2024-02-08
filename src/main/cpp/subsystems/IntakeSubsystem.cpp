
#include "subsystems/IntakeSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol;

IntakeSubsystem::IntakeSubsystem() 
    : m_motor(kIntakeRollerCANID)
#ifdef OVERUNDER
    , m_deployMotor(kIntakeDeployCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
#endif
{
    
    m_motor.SetNeutralMode(NeutralMode::Coast);

#ifdef OVERUNDER
  m_deployMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_deployMotor.SetClosedLoopRampRate(0.0);

  frc::Preferences::InitDouble("kIntakeDeployP", 1.0);
  frc::Preferences::InitDouble("kIntakeDeployI", 0.0);
  frc::Preferences::InitDouble("kIntakeDeployD", 0.0);

  m_deployMotorPIDController.SetP(frc::Preferences::GetDouble("kIntakeDeployP", 1.0));
  m_deployMotorPIDController.SetI(frc::Preferences::GetDouble("kIntakeDeployI", 0.0));
  m_deployMotorPIDController.SetD(frc::Preferences::GetDouble("kIntakeDeployD", 0.0));
  m_deployMotorPIDController.SetOutputRange(-1.0, 1.0);
#endif
}

void IntakeSubsystem::Periodic()
{
    SmartDashboard::PutNumber("D_I_Motor", m_motor.GetMotorOutputVoltage());
}

void IntakeSubsystem::Set(double speed)
{
    m_motor.Set(ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::ExtendIntake()
{
#ifdef OVERUNDER
    m_deployMotor.Set(0.0);
#endif


}

void IntakeSubsystem::RetractIntake()
{
#ifdef OVERUNDER
    m_deployMotor.Set(0.0);
#endif


}
