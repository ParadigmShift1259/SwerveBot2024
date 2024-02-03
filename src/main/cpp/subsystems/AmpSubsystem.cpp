#include "subsystems/AmpSubsystem.h"

AmpSubsystem::AmpSubsystem()
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_log = wpi::log::DoubleLogEntry(log, "/subsystem/amp");
}

void AmpSubsystem::Periodic()
{
  // m_log.Append(add_data_here);
}