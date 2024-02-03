#include "subsystems/FeederSubsystem.h"

FeederSubsystem::FeederSubsystem()
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_log = wpi::log::DoubleLogEntry(log, "/subsystem/feeder");
}

void FeederSubsystem::Periodic()
{
  // m_log.Append(add_data_here);
}