
#include "ConstantsDigitalInputs.h"
#include "subsystems/IntakeSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

constexpr double c_defaultIntakeP = 0.07;
constexpr double c_defaultIntakeI = 0.0;
constexpr double c_defaultIntakeD = 0.0;

constexpr double c_defaultRetractTurns = 0.0476;
constexpr double c_defaultExtendTurns = 13.3;
constexpr double c_defaultAdjustTurns = 1.5;

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

    frc::Preferences::InitDouble("kIntakeDeployP", c_defaultIntakeP);
    frc::Preferences::InitDouble("kIntakeDeployI", c_defaultIntakeI);
    frc::Preferences::InitDouble("kIntakeDeployD", c_defaultIntakeD);

    m_deployPIDController.SetOutputRange(kMinOut, kMaxOut);

    frc::SmartDashboard::PutNumber("DepRtctTurns", c_defaultRetractTurns);
    frc::SmartDashboard::PutNumber("DepExtTurns", c_defaultExtendTurns);
    frc::SmartDashboard::PutNumber("DepAdjTurns", c_defaultAdjustTurns);
#endif
}

void IntakeSubsystem::Periodic()
{
  static double lastP = 0.0;
  static double lastI = 0.0;
  static double lastD = 0.0;

//   static int count = 0;
//   if (count++ % 100 == 0)
  {
    auto p = frc::Preferences::GetDouble("kIntakeDeployP", c_defaultIntakeP);
    auto i = frc::Preferences::GetDouble("kIntakeDeployI", c_defaultIntakeI);
    auto d = frc::Preferences::GetDouble("kIntakeDeployD", c_defaultIntakeD);
    if (p != lastP)
    {
        m_deployPIDController.SetP(p);
    }
    if (i != lastI)
    {
        m_deployPIDController.SetI(i);
    }
    if (d != lastD)
    {
        m_deployPIDController.SetD(d);
    }
    lastP = p;
    lastI = i;
    lastD = d;
    
    frc::SmartDashboard::PutNumber("Deploy echo", m_deployRelativeEnc.GetPosition());
    frc::SmartDashboard::PutBoolean("Intake PhotoEye", m_photoEye.Get());
  }
}

void IntakeSubsystem::Set(double speed)
{
    m_motor.Set(ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::ExtendIntake()
{
#ifdef OVERUNDER
    double turns = frc::SmartDashboard::GetNumber("DepExtTurns", c_defaultExtendTurns);
    //double turns = 9.142;
    printf("dep turns %.3f\n", turns);
    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
    frc::SmartDashboard::PutNumber("DepApplOut", m_deployMotor.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("DepBusV", m_deployMotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("DepTemp", m_deployMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("DepOutAmps", m_deployMotor.GetOutputCurrent());    
#endif
}

void IntakeSubsystem::AdjustIntake()
{
#ifdef OVERUNDER
    double turns = frc::SmartDashboard::GetNumber("DepAdjTurns", c_defaultAdjustTurns);
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
    double turns = frc::SmartDashboard::GetNumber("DepRtctTurns", c_defaultRetractTurns);
    printf("dep turns %.3f\n", turns);
    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
#endif
}
