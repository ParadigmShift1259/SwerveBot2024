#include "subsystems/ShooterSubsystem.h"

#include <frc/Preferences.h>

ShooterSubsystem::ShooterSubsystem()
  : m_OverWheels(12, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_UnderWheels(13, rev::CANSparkLowLevel::MotorType::kBrushless)
#ifdef OVERUNDER  
  , m_BackWheels(1, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_ElevationController(2, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_ElevationEncoder(1)
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

  m_UnderPIDController.SetP(frc::Preferences::GetDouble("kShooterP", 1.0));
  m_UnderPIDController.SetI(frc::Preferences::GetDouble("kShooterI", 0.0));
  m_UnderPIDController.SetD(frc::Preferences::GetDouble("kShooterD", 0.0));

#ifdef OVERUNDER
  m_BackPIDController.SetP(frc::Preferences::GetDouble("kShooterP", 1.0));
  m_BackPIDController.SetI(frc::Preferences::GetDouble("kShooterI", 0.0));
  m_BackPIDController.SetD(frc::Preferences::GetDouble("kShooterD", 0.0));

  m_ElevationPIDController.SetP(frc::Preferences::GetDouble("kElevationP", 1.0));
  m_ElevationPIDController.SetI(frc::Preferences::GetDouble("kElevationI", 0.0));
  m_ElevationPIDController.SetD(frc::Preferences::GetDouble("kElevationD", 0.0));
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
  double overRPM = 1500.0;
  double underRPM = 1500.0;
#ifdef OVERUNDER  
  double backRPM = 1500.0;
#endif

  m_OverWheels.Set(0.5);
  m_UnderWheels.Set(-0.5);
  //m_OverPIDController.SetReference(overRPM, rev::CANSparkBase::ControlType::kVelocity);
  //m_UnderPIDController.SetReference(underRPM, rev::CANSparkBase::ControlType::kVelocity);
#ifdef OVERUNDER  
  m_BackPIDController.SetReference(backRPM, rev::CANSparkBase::ControlType::kVelocity);
#endif
}

void ShooterSubsystem::Stop()
{
#ifdef OVERUNDER
  m_BackWheels.StopMotor();
#endif
  m_OverWheels.StopMotor();
  m_UnderWheels.StopMotor();
}
    