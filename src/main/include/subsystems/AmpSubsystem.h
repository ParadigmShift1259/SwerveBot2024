#pragma once

#include <wpi/DataLog.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DataLogManager.h>

class AmpSubsystem : public frc2::SubsystemBase
{
  public:
    AmpSubsystem();
    void Periodic();
    
  private:
	wpi::log::DoubleLogEntry m_log;
};