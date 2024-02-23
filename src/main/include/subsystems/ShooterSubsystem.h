#pragma once

#include "Constants.h"
#include "PigeonGyro.h"

#include <wpi/DataLog.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DataLogManager.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>

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
    void GoToElevation(int shootIndex);
    void StartOverAndUnder(units::meter_t distance);
    void Shoot(units::meter_t distance);
    void Stop();
    bool GetLimitFront() const { return m_elevationLimitFront.Get(); }
    bool GetLimitRear() const { return m_elevationLimitRear.Get(); }
    units::degree_t GetCloseAngle() const { return m_closeAngle; }
    const std::vector<std::vector<double>> GetReferenceTable() const { return m_shootReference; }
    
  private:
    units::degree_t m_closeAngle;
    double m_elevationAngle = 0.0;
    double m_elevationTurns = 0.0;

    bool m_poppedPin = false;
    frc::Timer m_timer;

    PigeonGyro m_gyro;

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
    rev::SparkRelativeEncoder m_BackRelativeEnc = m_BackWheels.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkRelativeEncoder m_ElevationRelativeEnc = m_ElevationController.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

    ctre::phoenix6::hardware::CANcoder m_ElevationEncoder;
    frc::DigitalInput m_elevationLimitFront;
    frc::DigitalInput m_elevationLimitRear;
    
#endif    

    double m_overRPM;
    double m_underRPM;
    double m_backRPM;

    int m_shootIndex;

    std::vector<std::vector<double>> m_shootReference = 
    {
        { 5000.0,  5000.0},
        { 5000.0,  5000.0},
        {-1000.0, -1700.0},
        {   49.0,    32.0}
    };

	wpi::log::DoubleLogEntry m_logOverRPM;
    wpi::log::DoubleLogEntry m_logUnderRPM;
#ifdef OVERUNDER
	wpi::log::DoubleLogEntry m_logBackRPM;
	wpi::log::DoubleLogEntry m_logCurrentAngle;
	wpi::log::DoubleLogEntry m_logCommandedAngle;
	wpi::log::DoubleLogEntry m_logAbsoluteAngle;
#endif
};