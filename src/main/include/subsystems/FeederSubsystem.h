#pragma once

#include <wpi/DataLog.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DataLogManager.h>

class FeederSubsystem : public frc2::SubsystemBase
{
  public:
    FeederSubsystem();
    void Periodic();
    
  private:
	wpi::log::DoubleLogEntry m_log;
};