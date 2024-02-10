#include "ConstantsCANIDs.h"

#include "subsystems/ShooterSubsystem.h"

#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>

ShooterSubsystem::ShooterSubsystem()
  : m_OverWheels(kShooterOverWheelsCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_UnderWheels(kShooterUnderWheelsCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
#ifdef OVERUNDER  
  , m_BackWheels(kShooterBackWheelsCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_ElevationController(kShooterElevationControllerCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_ElevationEncoder(kElevationEncoderCANID)
#endif
{
  //m_OverWheels.RestoreFactoryDefaults();
  //m_UnderWheels.RestoreFactoryDefaults();
  //m_BackWheels.RestoreFactoryDefaults();

  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_logOverRPM = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/OverRPM");
  m_logUnderRPM = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/UnderRPM");
#ifdef OVERUNDER
  m_logBackRPM = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/BackRPM");
  m_logCurrentAngle = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/CurrentAngle");
  m_logCommandedAngle = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/CommandedAngle");
  m_logAbsoluteAngle = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/AbsoluteAngle");

  auto result = m_ElevationEncoder.GetAbsolutePosition();
  m_ElevationRelativeEnc.SetPosition(result.GetValueAsDouble());

  m_OverWheels.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_OverWheels.SetClosedLoopRampRate(0.0);
  m_OverWheels.SetInverted(false);
  m_UnderWheels.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_UnderWheels.SetClosedLoopRampRate(0.0);
  m_UnderWheels.SetInverted(false);

  frc::Preferences::InitDouble("kElevationP", 0.0005);
  frc::Preferences::InitDouble("kElevationI", 0.0);
  frc::Preferences::InitDouble("kElevationD", 0.0);
#endif
  frc::Preferences::InitDouble("kShooterP", 0.0005);
  frc::Preferences::InitDouble("kShooterI", 0.0);
  frc::Preferences::InitDouble("kShooterD", 0.0);
  frc::Preferences::InitDouble("kShooterFF", 0.0001);

  frc::Preferences::InitDouble("kShootVortexP", 0.0009);
  frc::Preferences::InitDouble("kShootVortexI", 0.0);
  frc::Preferences::InitDouble("kShootVortexD", 0.0);
  frc::Preferences::InitDouble("kShootVortexFF", 0.0001);

  m_OverPIDController.SetP(frc::Preferences::GetDouble("kShooterP", 0.0005));
  m_OverPIDController.SetI(frc::Preferences::GetDouble("kShooterI", 0.0));
  m_OverPIDController.SetD(frc::Preferences::GetDouble("kShooterD", 0.0));
  m_OverPIDController.SetOutputRange(kMinOut, kMaxOut);

  m_UnderPIDController.SetP(frc::Preferences::GetDouble("kShooterP", 0.0005));
  m_UnderPIDController.SetI(frc::Preferences::GetDouble("kShooterI", 0.0));
  m_UnderPIDController.SetD(frc::Preferences::GetDouble("kShooterD", 0.0));
  m_UnderPIDController.SetOutputRange(kMinOut, kMaxOut);

#ifdef OVERUNDER
  m_BackPIDController.SetP(frc::Preferences::GetDouble("kShootVortexP", 0.0009));
  m_BackPIDController.SetI(frc::Preferences::GetDouble("kShootVortexI", 0.0));
  m_BackPIDController.SetD(frc::Preferences::GetDouble("kShootVortexD", 0.0));
  m_BackPIDController.SetOutputRange(kMinOut, kMaxOut);

  m_ElevationPIDController.SetP(frc::Preferences::GetDouble("kElevationP", 0.0005));
  m_ElevationPIDController.SetI(frc::Preferences::GetDouble("kElevationI", 0.0));
  m_ElevationPIDController.SetD(frc::Preferences::GetDouble("kElevationD", 0.0));
  m_ElevationPIDController.SetOutputRange(kMinOut, kMaxOut);
#endif

  frc::SmartDashboard::PutNumber("OverRPM", -3000.0);
  frc::SmartDashboard::PutNumber("UnderRPM", 3000.0);
  frc::SmartDashboard::PutNumber("BackRPM", -3000.0);
}

void ShooterSubsystem::Periodic()
{
  // m_log.Append(add_data_here);
  m_logOverRPM.Append(m_OverRelativeEnc.GetVelocity()); 
  m_logUnderRPM.Append(m_UnderRelativeEnc.GetVelocity());
#ifdef OVERUNDER
  m_logBackRPM.Append(m_BackRelativeEnc.GetVelocity());
  m_logCurrentAngle.Append(m_ElevationRelativeEnc.GetPosition());
  m_logCommandedAngle.Append(m_elevationAngle);
  m_logAbsoluteAngle.Append(m_ElevationEncoder.GetAbsolutePosition().GetValueAsDouble());
#endif

  m_overRPM = frc::SmartDashboard::GetNumber("OverRPM", -3000.0);
  m_underRPM = frc::SmartDashboard::GetNumber("UnderRPM", 3000.0);
  m_backRPM = frc::SmartDashboard::GetNumber("BackRPM", -3000.0);

  frc::SmartDashboard::PutNumber("OverRPM echo", m_OverRelativeEnc.GetVelocity());
  frc::SmartDashboard::PutNumber("UnderRPM echo", m_UnderRelativeEnc.GetVelocity());
  frc::SmartDashboard::PutNumber("BackRPM echo", m_BackRelativeEnc.GetVelocity());
}

#ifdef OVERUNDER
void ShooterSubsystem::GoToElevation(units::degree_t angle)
{
  if (angle < 45.0_deg)
  {
    m_elevationAngle = 0.0; //TODO
  
    m_ElevationPIDController.SetReference(m_elevationAngle, rev::CANSparkBase::ControlType::kPosition);
  }
}
#endif

void ShooterSubsystem::Shoot(units::meter_t distance)
{
  //m_OverPIDController.SetIAccum(0);
  //m_UnderPIDController.SetIAccum(0);
  
//   double overRPM = 100.0;
//   double underRPM = -100.0;
// #ifdef OVERUNDER  
//   double backRPM = -100.0;
// #endif

  double ffNeo = frc::Preferences::GetDouble("kShooterFF", 0.0001);
  double ffVtx = frc::Preferences::GetDouble("kShootVortexFF", 0.0001);
  m_OverPIDController.SetFF(ffNeo);
  m_UnderPIDController.SetFF(ffNeo);

  printf("over %.3f under %.3f back %.3f \n", m_overRPM, m_underRPM, m_backRPM);
  //m_OverWheels.Set(0.5);
  //m_UnderWheels.Set(-0.5);
  m_OverPIDController.SetReference(m_overRPM, rev::CANSparkBase::ControlType::kVelocity);//, 0, ffNeo, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
  m_UnderPIDController.SetReference(m_underRPM, rev::CANSparkBase::ControlType::kVelocity);//, 0, ffNeo, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
#ifdef OVERUNDER  
  m_BackPIDController.SetFF(ffVtx);
  //m_BackWheels.Set(m_backRPM);
  m_BackPIDController.SetReference(m_backRPM, rev::CANSparkBase::ControlType::kVelocity);//, 0, ffVtx, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
#endif
}

void ShooterSubsystem::Stop()
{
#ifdef OVERUNDER
  m_BackWheels.StopMotor();
#endif
  m_OverWheels.StopMotor();
  m_UnderWheels.StopMotor();
}
    