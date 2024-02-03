#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem()
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_log = wpi::log::DoubleLogEntry(log, "/subsystem/shooter");
}

void ShooterSubsystem::Periodic()
{
  // m_log.Append(add_data_here);
}