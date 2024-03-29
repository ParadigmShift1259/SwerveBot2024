// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <string>
#include <wpi/DataLog.h>

#include <frc/Encoder.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Timer.h>
#include <frc/DataLogManager.h>

#include <networktables/NetworkTableEntry.h>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

// using namespace ctre::phoenix6::hardware::can;
using namespace ctre::phoenix6::hardware;
using namespace rev;

class SwerveModule
{
public:
    SwerveModule(int driveMotorCanId, int turningMotorCanId, double offset, bool driveMotorReversed);
    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();
    void SetDesiredState(const frc::SwerveModuleState& state);
    void Periodic();
    void ResyncAbsRelEnc();
    void SetMaxSpeed(units::meters_per_second_t newMaxSpeed) { m_currentMaxSpeed = newMaxSpeed; }

private:
    units::meters_per_second_t CalcMetersPerSec();
    units::meter_t CalcMeters();
    //double CalcTicksPer100Ms(units::meters_per_second_t speed);

    //static constexpr double kWheelRadius = 0.0508;
    static constexpr int kEncoderResolution = 4096;
    static constexpr double kTurnMotorRevsPerWheelRev = 150.0 / 7.0; //!< The steering gear ratio of the MK4i is 150/7:1
    //static constexpr double kTurnMotorRevsPerWheelRev = 12.8;        //!< The steering gear ratio of the MK4 is 12.8:1

    static constexpr auto kModuleMaxAngularVelocity = std::numbers::pi * 1_rad_per_s;  // radians per second
    static constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

    // https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/
    // static constexpr double kEncoderCPR = 2048.0;                  //!< Falcon internal encoder is 2048 CPR
    // static constexpr double kEncoderTicksPerSec = 10.0;            //!< TalonFX::GetSelectedSensorVelocity() returns ticks/100ms = 10 ticks/sec
    // static constexpr units::angular_velocity::turns_per_second_t kEncoderRevPerSec = units::angular_velocity::turns_per_second_t(kEncoderTicksPerSec / kEncoderCPR);  //!< CPR counts per rev is the same as ticks per rev
    static constexpr units::meter_t kWheelDiameterMeters = 4_in;          //!< 4"
    static constexpr units::meter_t kWheelCircumfMeters = kWheelDiameterMeters * std::numbers::pi;  //!< Distance driven per wheel rev

    // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    // The MK4i is available in 3 different drive gear ratios. 
    // The table below shows the drive gear ratios and free speeds with NEO and Falcon 500 motors. 
    // L1 and L2 ratios are the most popular ratios and are suitable for standard full weight competition robots. 
    // The L3 ratio is more aggressive and is recommended for light weight robots.
    static constexpr double kDriveGearRatioL1 = 8.14;    //!< MK4i swerve modules L1 gearing w/Falcon 13.5 ft/sec
    static constexpr double kDriveGearRatioL2 = 6.75;    //!< MK4i swerve modules L2 gearing w/Falcon 16.3 ft/sec
    static constexpr double kDriveGearRatioL3 = 6.12;    //!< MK4i swerve modules L3 gearing w/Falcon 18.0 ft/sec
    static constexpr double kDriveGearRatio = kDriveGearRatioL3;
    /// Assumes the encoders are mounted on the motor shaft
    /// ticks / 100 ms -> ticks / s -> motor rev / s -> wheel rev / s -> m / s
    // static constexpr units::meters_per_second_t kDriveEncoderMetersPerSec = kEncoderRevPerSec / kDriveGearRatio * kWheelCircumfMeters;
    // static constexpr double kDriveEncoderMetersPerTurn = kWheelCircumfMeters.to<double>() / (kEncoderCPR * kDriveGearRatio);
    units::meters_per_second_t m_currentMaxSpeed = 1.0_mps;

    TalonFX m_driveMotor;
    CANSparkMax m_turningMotor;

    std::string m_id;
    bool m_driveMotorReversed = false;

    SparkMaxAlternateEncoder m_turningEncoder = m_turningMotor.GetAlternateEncoder(SparkMaxAlternateEncoder::Type::kQuadrature, kEncoderResolution);

    frc::DutyCycleEncoder m_absEnc;
    double m_offset = 0.0;
    
    SparkPIDController m_turningPIDController = m_turningMotor.GetPIDController();

    frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V, 3_V / 1_mps};

    /// Timer used to sync absolute and relative encoders on robot turn on
    frc::Timer m_timer;

    // Logging Member Variables
    wpi::log::DoubleLogEntry m_logTurningEncoderPosition;
    wpi::log::DoubleLogEntry m_logAbsoluteEncoderPosition;
    wpi::log::DoubleLogEntry m_logAbsoluteEncoderPositionWithOffset;
    wpi::log::DoubleLogEntry m_logTurningRefSpeed;
    wpi::log::DoubleLogEntry m_logTurningRefAngle;
    wpi::log::DoubleLogEntry m_logTurningNewAngle;
    wpi::log::DoubleLogEntry m_logDriveNewSpeed;
    wpi::log::DoubleLogEntry m_logDriveNormalizedSpeed;

    std::array<std::string, 4> m_nameArray = 
    {
        "FrontLeft"
    ,   "FrontRight"
    ,   "RearRight"
    ,   "RearLeft"
    };
};
