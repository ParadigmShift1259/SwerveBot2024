#pragma once

#include <wpi/DataLog.h>

#include <ConstantsCANIDs.h>

#include <ctre/phoenix/led/CANdle.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DataLogManager.h>
#include <frc/AddressableLED.h>


using namespace ctre::phoenix::led;

class LEDSubsystem : public frc2::SubsystemBase
{
  public:
    LEDSubsystem();
    void Periodic();
    
  private:

    CANdleConfiguration m_candleConfig;
    wpi::log::DoubleLogEntry m_log;
    CANdle m_candle{kLEDCANId};
};