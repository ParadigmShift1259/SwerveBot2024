#pragma once

#include <wpi/DataLog.h>

#include <ConstantsDigitalInputs.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DataLogManager.h>
#include <frc/AddressableLED.h>

class LEDSubsystem : public frc2::SubsystemBase
{
  public:
    LEDSubsystem();
    void Periodic();
    
  private:
    wpi::log::DoubleLogEntry m_log;
    frc::AddressableLED m_ledStrip{kLEDPWMPort};

};