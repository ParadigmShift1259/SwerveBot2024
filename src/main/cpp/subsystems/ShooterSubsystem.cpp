#include "ConstantsCANIDs.h"

#include "subsystems/ShooterSubsystem.h"

#include <frc/Preferences.h>

ShooterSubsystem::ShooterSubsystem()
  : m_OverWheels(kShooterOverWheelsCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_UnderWheels(kShooterUnderWheelsCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
#ifdef OVERUNDER  
  , m_BackWheels(kShooterBackWheelsCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_ElevationController(kShooterElevationControllerCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_ElevationEncoder(kElevationEncoderCANID)
#endif
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_logOverRPM = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/OverRPM");
  m_logUnderRPM = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/UnderRPM");
#ifdef OVERUNDER
  m_logBackRPM = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/BackRPM");
  m_logCurrentAngle = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/CurrentAngle");
  m_logCommandedAngle = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/CommandedAngle");
  m_logAbsoluteAngle = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/AbsoluteAngle");

  auto result = m_ElevationEncoder.GetAbsolutePosition();
  m_ElevationRelativeEnc.SetPosition(result.GetValueAsDouble());

  m_OverWheels.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_OverWheels.SetClosedLoopRampRate(0.0);
  m_UnderWheels.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_UnderWheels.SetClosedLoopRampRate(0.0);

  frc::Preferences::InitDouble("kElevationP", 1.0);
  frc::Preferences::InitDouble("kElevationI", 0.0);
  frc::Preferences::InitDouble("kElevationD", 0.0);
#endif
  frc::Preferences::InitDouble("kShooterP", 1.0);
  frc::Preferences::InitDouble("kShooterI", 0.0);
  frc::Preferences::InitDouble("kShooterD", 0.0);

  m_OverPIDController.SetP(frc::Preferences::GetDouble("kShooterP", 1.0));
  m_OverPIDController.SetI(frc::Preferences::GetDouble("kShooterI", 0.0));
  m_OverPIDController.SetD(frc::Preferences::GetDouble("kShooterD", 0.0));
  m_OverPIDController.SetOutputRange(kMinOut, kMaxOut);

  m_UnderPIDController.SetP(frc::Preferences::GetDouble("kShooterP", 1.0));
  m_UnderPIDController.SetI(frc::Preferences::GetDouble("kShooterI", 0.0));
  m_UnderPIDController.SetD(frc::Preferences::GetDouble("kShooterD", 0.0));
  m_UnderPIDController.SetOutputRange(kMinOut, kMaxOut);

#ifdef OVERUNDER
  m_BackPIDController.SetP(frc::Preferences::GetDouble("kShooterP", 1.0));
  m_BackPIDController.SetI(frc::Preferences::GetDouble("kShooterI", 0.0));
  m_BackPIDController.SetD(frc::Preferences::GetDouble("kShooterD", 0.0));
  m_BackPIDController.SetOutputRange(kMinOut, kMaxOut);

  m_ElevationPIDController.SetP(frc::Preferences::GetDouble("kElevationP", 1.0));
  m_ElevationPIDController.SetI(frc::Preferences::GetDouble("kElevationI", 0.0));
  m_ElevationPIDController.SetD(frc::Preferences::GetDouble("kElevationD", 0.0));
  m_ElevationPIDController  .SetOutputRange(kMinOut, kMaxOut);
#endif
}

void ShooterSubsystem::Periodic()
{
  // m_log.Append(add_data_here);
  m_logOverRPM.Append(m_OverRelativeEnc.GetVelocity()); 
  m_logUnderRPM.Append(m_UnderRelativeEnc.GetVelocity());
#ifdef OVERUNDER
  m_logBackRPM.Append(m_BackRelativeEnc.GetVelocity());
  m_logCurrentAngle.Append(m_ElevationRelativeEnc.GetPosition());
  m_logCommandedAngle.Append(m_elevationAngle);
  m_logAbsoluteAngle.Append(m_ElevationEncoder.GetAbsolutePosition().GetValueAsDouble());
#endif
}

#ifdef OVERUNDER
void ShooterSubsystem::GoToElevation(units::degree_t angle)
{
  if (angle < 45.0_deg)
  {
    m_elevationAngle = 0.0; //TODO
  
    m_ElevationPIDController.SetReference(m_elevationAngle, rev::CANSparkBase::ControlType::kPosition);
  }
}
#endif

void ShooterSubsystem::Shoot(units::meter_t distance)
{
  //m_OverPIDController.SetIAccum(0);
  //m_UnderPIDController.SetIAccum(0);
  static bool bFirstTime = false;
  
  double overRPM = 1500.0;
  double underRPM = 1500.0;
#ifdef OVERUNDER  
  double backRPM = -0.5;
#endif

  m_OverWheels.Set(0.5);
  m_UnderWheels.Set(-0.5);
  //m_OverPIDController.SetReference(overRPM, rev::CANSparkBase::ControlType::kVelocity);
  //m_UnderPIDController.SetReference(underRPM, rev::CANSparkBase::ControlType::kVelocity);
#ifdef OVERUNDER  
  //m_BackWheels.Set(-0.5);
  if (!bFirstTime)
  {
    m_BackPIDController.SetReference(backRPM, rev::CANSparkBase::ControlType::kVelocity);
  }
#endif
  bFirstTime = true;
}

void ShooterSubsystem::Stop()
{
#ifdef OVERUNDER
  m_BackWheels.StopMotor();
#endif
  m_OverWheels.StopMotor();
  m_UnderWheels.StopMotor();
}
    