#pragma once

#include "Constants.h"
#include "PigeonGyro.h"

#include <wpi/DataLog.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DataLogManager.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>

#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>

#include <units/angle.h>
#include <units/length.h>

constexpr units::angle::degree_t c_defaultShootFarAngle = 36.0_deg;
constexpr units::angle::degree_t c_defaultShootCloseAngle = 55.0_deg;

class ShooterSubsystem : public frc2::SubsystemBase
{
  public:
    ShooterSubsystem();
    void Periodic() override;
    void GoToElevation(units::degree_t angle);
    void GoToElevation(int shootIndex);
    void StartOverAndUnder(units::meter_t distance);
    void Shoot(units::meter_t distance);
    void Stop();
    void EnableSyncToGyro() { m_bSyncToGyro = true; }
    void DisableSyncToGyro() { m_bSyncToGyro = false; }
    units::degree_t GetCloseAngle() const { return m_closeAngle; }
    const std::vector<std::vector<double>> GetReferenceTable() const { return m_shootReference; }
    
  private:
    units::degree_t m_closeAngle;
    double m_elevationAngle = 55.0;
    double m_elevationTurns = 0.0;  // For calibration
    double m_lastElevationTurns = 0.0;  // For calibration
    bool m_bSyncToGyro = true;
    frc::Timer m_timer;

    PigeonGyro m_gyro;

#ifdef THING1
    rev::CANSparkMax m_OverWheels;
    rev::CANSparkMax m_UnderWheels;
#else
    rev::CANSparkFlex m_OverWheels;
    rev::CANSparkFlex m_UnderWheels; 
#endif

    rev::CANSparkMax m_ElevationController;

    rev::SparkPIDController m_OverPIDController = m_OverWheels.GetPIDController();
    rev::SparkPIDController m_UnderPIDController = m_UnderWheels.GetPIDController();
    rev::SparkPIDController m_ElevationPIDController = m_ElevationController.GetPIDController();

    rev::SparkRelativeEncoder m_OverRelativeEnc = m_OverWheels.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
    rev::SparkRelativeEncoder m_UnderRelativeEnc = m_UnderWheels.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
    rev::SparkRelativeEncoder m_ElevationRelativeEnc = m_ElevationController.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);

    double m_overRPM;
    double m_underRPM;
    double m_backRPM;

    int m_shootIndex;

    std::vector<std::vector<double>> m_shootReference = 
    {
        // Near    Far
        { 3800.0,  3800.0}, // Over/Under (Left/Right) RPM
        {    0.0,     0.0}, // Back wheels (not used in SxS) RPM
        {   c_defaultShootCloseAngle.value(),    c_defaultShootFarAngle.value() }  // Shot angle for Speaker and Podium
    };

	wpi::log::DoubleLogEntry m_logOverRPM;
    wpi::log::DoubleLogEntry m_logUnderRPM;
	wpi::log::DoubleLogEntry m_logCurrentAngle;
	wpi::log::DoubleLogEntry m_logCommandedAngle;
	wpi::log::DoubleLogEntry m_logAbsoluteAngle;
    wpi::log::DoubleLogEntry m_logElevTurns;
};