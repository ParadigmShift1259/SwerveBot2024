// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(const int driveMotorCanId, const int turningMotorCanId, double offset, bool driveMotorReversed)
  : m_driveMotor(driveMotorCanId)
  , m_turningMotor(turningMotorCanId, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_id(std::to_string(turningMotorCanId / 2))
  , m_absEnc((turningMotorCanId / 2) - 1)
  , m_offset(offset)
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  std::string logHeader = "/swerveModule" + m_id + "/";
  m_logTurningEncoderPosition = wpi::log::DoubleLogEntry(log, logHeader + "turningEncoderPosition");
  m_logAbsoluteEncoderPosition = wpi::log::DoubleLogEntry(log, logHeader + "absoluteEncoderPosition");
  m_logAbsoluteEncoderPositionWithOffset = wpi::log::DoubleLogEntry(log, logHeader + "absoluteEncoderPositionWithOffset");
  m_logTurningRefSpeed = wpi::log::DoubleLogEntry(log, logHeader + "refSpeed");
  m_logTurningRefAngle = wpi::log::DoubleLogEntry(log, logHeader + "refAngle");
  m_logTurningNewAngle = wpi::log::DoubleLogEntry(log, logHeader + "newAngle");
  m_logDriveNewSpeed = wpi::log::DoubleLogEntry(log, logHeader + "newSpeed");
  m_logDriveNormalizedSpeed = wpi::log::DoubleLogEntry(log, logHeader + "normalizedNewSpeed");

  m_turningEncoder.SetInverted(true);               // SDS Mk4i motors are mounted upside down compared to the Mk4
  m_turningEncoder.SetPositionConversionFactor(2.0 * std::numbers::pi); //<! Converts from wheel rotations to radians

  m_absEnc.SetConnectedFrequencyThreshold(200);
  auto angle = fmod(1 + m_offset - m_absEnc.GetAbsolutePosition(), 1.0);
  m_turningEncoder.SetPosition(angle * 2 * std::numbers::pi);

  ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitConfigs;
  currentLimitConfigs.WithStatorCurrentLimit(60);
  currentLimitConfigs.WithStatorCurrentLimitEnable(true);
  currentLimitConfigs.WithSupplyCurrentLimit(60);
  currentLimitConfigs.WithSupplyCurrentLimitEnable(true);
  currentLimitConfigs.WithSupplyCurrentThreshold(70);
  currentLimitConfigs.WithSupplyTimeThreshold(0.85);
  m_driveMotor.SetInverted(driveMotorReversed);
  // m_driveMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
  m_driveMotor.SetPosition(units::angle::turn_t(0.0));
  m_driveMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

  constexpr double kDriveP = 0.0025; // 0.1;
  constexpr double kDriveI = 0;
  constexpr double kDriveD = 0;
  // constexpr double kDriveFF = 0.055;//0.047619;
  constexpr double m_max = 1.0;
  constexpr double m_min = -1.0;

  ctre::phoenix6::configs::Slot0Configs slot0Configs;
  slot0Configs.WithKP(kDriveP);
  slot0Configs.WithKI(kDriveI);
  slot0Configs.WithKD(kDriveD);
  // m_driveMotor.Config_kF(0, kDriveFF);

  ctre::phoenix6::configs::MotorOutputConfigs motorOutputConfigs;
  motorOutputConfigs.WithPeakForwardDutyCycle(m_max);
  motorOutputConfigs.WithPeakReverseDutyCycle(m_min);

  m_driveMotor.GetConfigurator().Apply(currentLimitConfigs);
  m_driveMotor.GetConfigurator().Apply(slot0Configs);
  m_driveMotor.GetConfigurator().Apply(motorOutputConfigs);

  m_turningPIDController.SetFeedbackDevice(m_turningEncoder);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.SetOutputRange(-1.0, 1.0);
  constexpr double kTurnP = 1.0;
  //constexpr double kTurnI = 0.000001;
  constexpr double kTurnI = 0.0;
  constexpr double kTurnD = 0.0;
  m_turningPIDController.SetP(kTurnP);
  m_turningPIDController.SetI(kTurnI);
  m_turningPIDController.SetD(kTurnD);
  frc::SmartDashboard::PutBoolean("Load Turn PID", false);
  frc::SmartDashboard::PutNumber("Turn P", kTurnP);
  frc::SmartDashboard::PutNumber("Turn I", kTurnI);
  frc::SmartDashboard::PutNumber("Turn D", kTurnD);
  m_turningMotor.SetSmartCurrentLimit(20);
  m_turningMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
  
  m_timer.Reset();
  m_timer.Start();
}

void SwerveModule::Periodic()
{
  auto time = m_timer.Get();
  if (time < 1.0_s)
  {
    ResyncAbsRelEnc();
  }

  // static int count = 0;
  // if (count++ % 5)
  // {
  //   ResyncAbsRelEnc();
  // }

  // Log relative encoder and absolute encoder positions (radians)
  auto absPos = m_absEnc.GetAbsolutePosition();
  auto angle = fmod(1 + m_offset - absPos, 1.0);
  m_logTurningEncoderPosition.Append(m_turningEncoder.GetPosition());
  //m_logAbsoluteEncoderPosition.Append(absPos * 2 * std::numbers::pi);
  m_logAbsoluteEncoderPosition.Append(((m_offset - absPos) * 2 * std::numbers::pi) - (m_turningEncoder.GetPosition()));
  m_logAbsoluteEncoderPositionWithOffset.Append((m_offset - absPos) * 2 * std::numbers::pi);

  bool bLoadPID = frc::SmartDashboard::GetBoolean("Load Turn PID", false);
  if (bLoadPID)
  {
    double kTurnP = frc::SmartDashboard::GetNumber("Turn P", 0.5);
    double kTurnI = frc::SmartDashboard::GetNumber("Turn I", 0.00001);
    double kTurnD = frc::SmartDashboard::GetNumber("Turn D", 0.05);
    m_turningPIDController.SetP(kTurnP);
    m_turningPIDController.SetI(kTurnI);
    m_turningPIDController.SetD(kTurnD);
    frc::SmartDashboard::PutNumber("Turn P echo", kTurnP);
    frc::SmartDashboard::PutNumber("Turn I echo", kTurnI);
    frc::SmartDashboard::PutNumber("Turn D echo", kTurnD);
    printf("Loaded Turn PID values P %.3f I %.3f D %.3f\n", kTurnP, kTurnI, kTurnD);
  }

  frc::SmartDashboard::PutNumber("Abs Pos" + m_id, absPos);
  frc::SmartDashboard::PutNumber("Abs Pos Offset" + m_id, angle);

  frc::SmartDashboard::PutNumber("Turn Enc Pos" + m_id, -1.0 * m_turningEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Turn Mot Pos" + m_id, -1.0 * m_turningEncoder.GetPosition() * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi));
}

void SwerveModule::ResyncAbsRelEnc()
{
  auto angleInRot = fmod(1 + m_offset - m_absEnc.GetAbsolutePosition(), 1.0); // Returns rotations between 0 and 1
  auto angleInRad = angleInRot * 2 * std::numbers::pi;                        // Changes rotations to radians
  if (angleInRad > std::numbers::pi)                                          // If angle is between pi and 2pi, put it between -pi and 0
      angleInRad -= 2 * std::numbers::pi;

  m_turningEncoder.SetPosition(angleInRad);
#define PRINT_ABS_RESYNC
#ifdef PRINT_ABS_RESYNC
  auto time = m_timer.Get();
  printf("Module %s %.3f Set abs enc %.3f [rot] %.3f [rad] to rel enc %.3f [rad] mot pos %.3f [rot]\n"
        , m_id.c_str()
        , time.to<double>()
        , angleInRot
        , angleInRad
        , -1.0 * m_turningEncoder.GetPosition()
        , -1.0 * m_turningEncoder.GetPosition() * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi));
#endif
}

frc::SwerveModuleState SwerveModule::GetState()
{
  return { CalcMetersPerSec(), units::radian_t{ -1.0 * m_turningEncoder.GetPosition() } };
}

units::meters_per_second_t SwerveModule::CalcMetersPerSec()
{
  return kWheelCircumfMeters * m_driveMotor.GetVelocity().GetValue() / units::angle::turn_t(1.0);
}

frc::SwerveModulePosition SwerveModule::GetPosition()
{
  return {CalcMeters(), units::radian_t{ -1.0 * m_turningEncoder.GetPosition() } };
}

units::meter_t SwerveModule::CalcMeters()
{
  return kWheelCircumfMeters * m_driveMotor.GetPosition().GetValue() / units::angle::turn_t(1.0);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState)
{
  // Optimize the reference state to avoid spinning further than 90 degrees
  double currPosition = -1.0 * m_turningEncoder.GetPosition();
  const auto state = frc::SwerveModuleState::Optimize(referenceState, frc::Rotation2d{ units::radian_t(currPosition) });
  //frc::SmartDashboard::PutNumber("Turn Enc Pos" + m_id, currPosition);
  //frc::SmartDashboard::PutNumber("Turn Mot Pos" + m_id, currPosition * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi));

  if (state.speed != 0_mps)
  {
#ifdef DISABLE_DRIVE
    m_driveMotor.Set(TalonFXControlMode::Velocity, 0.0);
#else
    // TODO Change how the speed to the drive motor is calculated
    // m_driveMotor.Set(TalonFXControlMode::Velocity, CalcTicksPer100Ms(state.speed));
    if (m_id == "1")
    {
      m_driveMotor.Set(1.0);
    }
    else
    {
      m_driveMotor.Set((state.speed / m_currentMaxSpeed).to<double>());
    }
#endif
  }
  else
  {
    m_driveMotor.Set(0.0);
  }

  // Calculate the turning motor output from the turning PID controller.
  //frc::SmartDashboard::PutNumber("Turn Ref Opt" + m_id, state.angle.Radians().to<double>());
  //frc::SmartDashboard::PutNumber("Turn Ref" + m_id, referenceState.angle.Radians().to<double>());
  double newRef = state.angle.Radians().to<double>();
 
  m_logTurningRefSpeed.Append(referenceState.speed.to<double>());
  m_logTurningRefAngle.Append(referenceState.angle.Degrees().to<double>());
  m_logTurningNewAngle.Append(state.angle.Degrees().to<double>());
  m_logDriveNewSpeed.Append(state.speed.to<double>());
  m_logDriveNormalizedSpeed.Append((state.speed / m_currentMaxSpeed).to<double>());

  //frc::SmartDashboard::PutNumber("Turn Ref Motor" + m_id, newRef);
  m_turningPIDController.SetReference(-1.0 * newRef, CANSparkMax::ControlType::kPosition);
}