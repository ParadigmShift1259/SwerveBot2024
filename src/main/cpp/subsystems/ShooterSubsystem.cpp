#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem()
  : m_OverWheels(12, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_UnderWheels(13, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_BackWheels(1, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_ElevationController(2, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_ElevationEncoder(1)
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_logOverRPM = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/OverRPM");
  m_logUnderRPM = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/UnderRPM");
  m_logBackRPM = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/BackRPM");
  m_logCurrentAngle = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/CurrentAngle");
  m_logCommandedAngle = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/CommandedAngle");
  m_logAbsoluteAngle = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/AbsoluteAngle");

  auto result = m_ElevationEncoder.GetAbsolutePosition();
  m_ElevationRelativeEnc.SetPosition(result.GetValueAsDouble());
}

void ShooterSubsystem::Periodic()
{
  // m_log.Append(add_data_here);
  m_logOverRPM.Append(m_OverRelativeEnc.GetVelocity()); 
  m_logUnderRPM.Append(m_UnderRelativeEnc.GetVelocity());
  m_logBackRPM.Append(m_BackRelativeEnc.GetVelocity());
  m_logCurrentAngle.Append(m_ElevationRelativeEnc.GetPosition());
  m_logCommandedAngle.Append(m_elevationAngle);
  m_logAbsoluteAngle.Append(m_ElevationEncoder.GetAbsolutePosition().GetValueAsDouble());

}

void ShooterSubsystem::GoToElevation(units::degree_t angle)
{
  if (angle < 45.0_deg)
  {
    m_elevationAngle = 0.0; //TODO
  
    m_ElevationPIDController.SetReference(m_elevationAngle, rev::CANSparkBase::ControlType::kPosition);
  }


}

void ShooterSubsystem::Shoot(units::meter_t distance)
{
  double overRPM = 500.0;
  double underRPM = 500.0;
  double backRPM = 500.0;

  m_OverPIDController.SetReference(overRPM, rev::CANSparkBase::ControlType::kVelocity);
  m_UnderPIDController.SetReference(underRPM, rev::CANSparkBase::ControlType::kVelocity);
  m_BackPIDController.SetReference(backRPM, rev::CANSparkBase::ControlType::kVelocity);

  
}

void ShooterSubsystem::Stop()
{
  m_BackWheels.StopMotor();
  m_OverWheels.StopMotor();
  m_UnderWheels.StopMotor();
}
    