#pragma once

// #define OVERUNDER

#include <wpi/DataLog.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DataLogManager.h>

#include <rev/CANSparkFlex.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <rev/CANSparkMax.h>

#include <units/angle.h>
#include <units/length.h>

class ShooterSubsystem : public frc2::SubsystemBase
{
  public:
    ShooterSubsystem();
    void Periodic();
    void GoToElevation(units::degree_t angle);
    void Shoot(units::meter_t distance);
    void Stop();
    
  private:
    double m_elevationAngle = 0.0;

    rev::CANSparkMax m_OverWheels;
    rev::CANSparkMax m_UnderWheels;
#ifdef OVERUNDER
    rev::CANSparkFlex m_BackWheels;
    rev::CANSparkFlex m_ElevationController; 
#endif

    rev::SparkPIDController m_OverPIDController = m_OverWheels.GetPIDController();
    rev::SparkPIDController m_UnderPIDController = m_UnderWheels.GetPIDController();
#ifdef OVERUNDER
    rev::SparkPIDController m_BackPIDController = m_BackWheels.GetPIDController();
    rev::SparkPIDController m_ElevationPIDController = m_ElevationController.GetPIDController();
#endif

    rev::SparkRelativeEncoder m_OverRelativeEnc = m_OverWheels.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
    rev::SparkRelativeEncoder m_UnderRelativeEnc = m_UnderWheels.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
#ifdef OVERUNDER    
    rev::SparkRelativeEncoder m_BackRelativeEnc = m_BackWheels.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
    rev::SparkRelativeEncoder m_ElevationRelativeEnc = m_ElevationController.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);    

    ctre::phoenix6::hardware::CANcoder m_ElevationEncoder;
#endif    

	  wpi::log::DoubleLogEntry m_logOverRPM;
    wpi::log::DoubleLogEntry m_logUnderRPM;
#ifdef OVERUNDER
	  wpi::log::DoubleLogEntry m_logBackRPM;
	  wpi::log::DoubleLogEntry m_logCurrentAngle;
	  wpi::log::DoubleLogEntry m_logCommandedAngle;
	  wpi::log::DoubleLogEntry m_logAbsoluteAngle;
#endif

};