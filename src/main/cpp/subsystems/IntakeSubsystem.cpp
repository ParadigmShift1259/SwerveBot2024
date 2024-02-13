
#include "ConstantsDigitalInputs.h"
#include "subsystems/IntakeSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol;

IntakeSubsystem::IntakeSubsystem() 
    : m_motor(kIntakeRollerCANID)
    , m_photoEye(kIntakePhotoeye)
#ifdef OVERUNDER
    , m_deployMotor(kIntakeDeployCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
#endif
{
  m_motor.SetNeutralMode(NeutralMode::Coast);

#ifdef OVERUNDER
    m_deployMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_deployMotor.SetClosedLoopRampRate(0.0);
    m_deployRelativeEnc.SetPosition(0.0);

    frc::Preferences::InitDouble("kIntakeDeployP", 1.0);
    frc::Preferences::InitDouble("kIntakeDeployI", 0.0);
    frc::Preferences::InitDouble("kIntakeDeployD", 0.0);

    m_deployPIDController.SetOutputRange(kMinOut, kMaxOut);

    frc::SmartDashboard::PutNumber("DepRtctTurns", 0.0476);
    frc::SmartDashboard::PutNumber("DepExtTurns", 9.1);
#endif
}

void IntakeSubsystem::Periodic()
{
  static int count = 0;
  if (count++ % 100 == 0)
  {
    m_deployPIDController.SetP(frc::Preferences::GetDouble("kIntakeDeployP", 1.0));
    m_deployPIDController.SetI(frc::Preferences::GetDouble("kIntakeDeployI", 0.0));
    m_deployPIDController.SetD(frc::Preferences::GetDouble("kIntakeDeployD", 0.0));
    frc::SmartDashboard::PutNumber("Deploy echo", m_deployRelativeEnc.GetPosition());
  }
}

void IntakeSubsystem::Set(double speed)
{
    m_motor.Set(ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::ExtendIntake()
{
#ifdef OVERUNDER
    double turns = frc::SmartDashboard::GetNumber("DepExtTurns", 9.1);
    //double turns = 9.142;
    printf("dep turns %.3f\n", turns);
    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
    frc::SmartDashboard::PutNumber("DepApplOut", m_deployMotor.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("DepBusV", m_deployMotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("DepTemp", m_deployMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("DepOutAmps", m_deployMotor.GetOutputCurrent());    
#endif
}

void IntakeSubsystem::RetractIntake()
{
#ifdef OVERUNDER
    double turns = frc::SmartDashboard::GetNumber("DepRtctTurns", 0.0476);
    //double turns = 0.0476;
    printf("dep turns %.3f\n", turns);
    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
#endif
}
