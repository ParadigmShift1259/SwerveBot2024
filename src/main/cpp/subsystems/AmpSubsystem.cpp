#include "subsystems/AmpSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

constexpr double c_defaultPlopperP = 0.07;
constexpr double c_defaultPlopperI = 0.0;
constexpr double c_defaultPlopperD = 0.0;

constexpr double c_defaultAmpTurns = 0.0;
constexpr double c_defaultParkTurns = 0.0;       
constexpr double c_defaultTransferTurns = 45.0;
constexpr double c_defaultTrapTurns = 45.0;

using namespace frc;
using namespace ctre::phoenix::motorcontrol;

AmpSubsystem::AmpSubsystem()
    : m_motor(kPlopperRollerCANID)
    // , m_photoEye(kIntakePhotoeye)
    , m_liftMotor(kPlopperElevationControllerCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_motor.SetNeutralMode(NeutralMode::Coast);

  m_liftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_liftMotor.SetClosedLoopRampRate(0.0);
  m_liftRelativeEnc.SetPosition(0.0);

  frc::Preferences::InitDouble("kPlopperPosP", c_defaultPlopperP);
  frc::Preferences::InitDouble("kPlopperPosI", c_defaultPlopperI);
  frc::Preferences::InitDouble("kPlopperPosD", c_defaultPlopperD);

  m_liftPIDController.SetOutputRange(kMinOut, kMaxOut);

  frc::SmartDashboard::PutNumber("LiftAmpTurns", c_defaultAmpTurns);
  frc::SmartDashboard::PutNumber("LiftParkTurns", c_defaultParkTurns);
  frc::SmartDashboard::PutNumber("LiftXfrTurns", c_defaultTransferTurns);
  frc::SmartDashboard::PutNumber("LiftTrapTurns", c_defaultTrapTurns);

  frc::SmartDashboard::PutNumber("PlopperPosition", 1.0);
  frc::SmartDashboard::PutNumber("PlopperSpeed", 0.0);

  m_log = wpi::log::DoubleLogEntry(log, "/subsystem/amp");
}

void AmpSubsystem::Periodic()
{
  // m_log.Append(add_data_here);
  static int count = 0;
  if (count++ % 50 == 0)
  {
    m_liftPIDController.SetP(frc::Preferences::GetDouble("kPlopperPosP", c_defaultPlopperP));
    m_liftPIDController.SetI(frc::Preferences::GetDouble("kPlopperPosI", c_defaultPlopperI));
    m_liftPIDController.SetD(frc::Preferences::GetDouble("kPlopperPosD", c_defaultPlopperD));
    frc::SmartDashboard::PutNumber("Plopper echo", m_liftRelativeEnc.GetPosition());

    m_AmpTurns      = frc::SmartDashboard::GetNumber("LiftAmpTurns", c_defaultAmpTurns);
    m_ParkTurns     = frc::SmartDashboard::GetNumber("LiftParkTurns", c_defaultParkTurns);
    m_TransferTurns = frc::SmartDashboard::GetNumber("LiftXfrTurns", c_defaultTransferTurns);
    m_TrapTurns     = frc::SmartDashboard::GetNumber("LiftTrapTurns", c_defaultTrapTurns);

    m_plopperPosition = frc::SmartDashboard::GetNumber("PlopperPosition", 1.0);
    m_plopperSpeed = frc::SmartDashboard::GetNumber("PlopperSpeed", 0.0);
  }
  // frc::SmartDashboard::PutBoolean("Plopper PhotoEye", m_photoEye.Get());
}

void AmpSubsystem::Set(double speed)
{
  printf("speed %.3f\n", m_plopperSpeed);
  m_motor.Set(ControlMode::PercentOutput, m_plopperSpeed);
}

void AmpSubsystem::GoToPosition(double position)
{

  double turns = m_plopperPosition;
  printf("plopper turns %.3f\n", turns);
  m_liftPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
}

