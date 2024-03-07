#include "subsystems/LEDSubsystem.h"

LEDSubsystem::LEDSubsystem()
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_log = wpi::log::DoubleLogEntry(log, "/subsystem/led");
}

void LEDSubsystem::Periodic()
{
  // m_log.Append(add_data_here);
}