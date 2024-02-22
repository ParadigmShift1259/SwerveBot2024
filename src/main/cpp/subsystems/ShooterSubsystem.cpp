#include "ConstantsCANIDs.h"
#include "ConstantsDigitalInputs.h"

#include "subsystems/ShooterSubsystem.h"

#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <hal/HAL.h>

constexpr double c_defaultRPM = 4500.0;
constexpr double c_defaultBackRPM = 1400.0;

constexpr double c_defaultShootNeoP = 0.0005;
constexpr double c_defaultShootNeoI = 0.0;
constexpr double c_defaultShootNeoD = 0.0;
constexpr double c_defaultShootNeoFF = 0.0001;

constexpr double c_defaultShootVortexP = 0.0009;
constexpr double c_defaultShootVortexI = 0.0;
constexpr double c_defaultShootVortexD = 0.0;
constexpr double c_defaultShootVortexFF = 0.0001;

constexpr double c_defaultElevP = 1.0;
constexpr double c_defaultElevI = 0.0;
constexpr double c_defaultElevD = 0.0;
constexpr double c_defaultElevFF = 0.07;

ShooterSubsystem::ShooterSubsystem()
  : m_gyro(kShooterPigeonCANID)
  , m_OverWheels(kShooterOverWheelsCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_UnderWheels(kShooterUnderWheelsCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
#ifdef OVERUNDER  
  , m_BackWheels(kShooterBackWheelsCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_ElevationController(kShooterElevationControllerCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_ElevationEncoder(kElevationEncoderCANID)
  , m_elevationLimitFront(kElevationLimitFront)
  , m_elevationLimitRear(kElevationLimitRear)
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
  m_ElevationRelativeEnc.SetPositionConversionFactor(1.0);

  m_OverWheels.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_OverWheels.SetClosedLoopRampRate(0.0);
  m_OverWheels.SetInverted(false);
  m_UnderWheels.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_UnderWheels.SetClosedLoopRampRate(0.0);
  m_UnderWheels.SetInverted(false);

  frc::Preferences::InitDouble("kElevationP", c_defaultElevP);
  frc::Preferences::InitDouble("kElevationI", c_defaultElevI);
  frc::Preferences::InitDouble("kElevationD", c_defaultElevD);
  frc::Preferences::InitDouble("kElevationFF", c_defaultElevFF);
#endif
  frc::Preferences::InitDouble("kShooterP", c_defaultShootNeoP);
  frc::Preferences::InitDouble("kShooterI", c_defaultShootNeoI);
  frc::Preferences::InitDouble("kShooterD", c_defaultShootNeoD);
  frc::Preferences::InitDouble("kShooterFF", c_defaultShootNeoFF);

  frc::Preferences::InitDouble("kShootVortexP", c_defaultShootVortexP);
  frc::Preferences::InitDouble("kShootVortexI", c_defaultShootVortexI);
  frc::Preferences::InitDouble("kShootVortexD", c_defaultShootVortexD);
  frc::Preferences::InitDouble("kShootVortexFF", c_defaultShootVortexFF);

  m_OverPIDController.SetP(frc::Preferences::GetDouble("kShooterP", c_defaultShootNeoP));
  m_OverPIDController.SetI(frc::Preferences::GetDouble("kShooterI", c_defaultShootNeoI));
  m_OverPIDController.SetD(frc::Preferences::GetDouble("kShooterD", c_defaultShootNeoD));
  m_OverPIDController.SetOutputRange(kMinOut, kMaxOut);

  m_UnderPIDController.SetP(frc::Preferences::GetDouble("kShooterP", c_defaultShootNeoP));
  m_UnderPIDController.SetI(frc::Preferences::GetDouble("kShooterI", c_defaultShootNeoI));
  m_UnderPIDController.SetD(frc::Preferences::GetDouble("kShooterD", c_defaultShootNeoD));
  m_UnderPIDController.SetOutputRange(kMinOut, kMaxOut);

#ifdef OVERUNDER
  m_BackPIDController.SetP(frc::Preferences::GetDouble("kShootVortexP", c_defaultShootVortexP));
  m_BackPIDController.SetI(frc::Preferences::GetDouble("kShootVortexI", c_defaultShootVortexI));
  m_BackPIDController.SetD(frc::Preferences::GetDouble("kShootVortexD", c_defaultShootVortexD));
  m_BackPIDController.SetOutputRange(kMinOut, kMaxOut);

  m_ElevationController.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_ElevationPIDController.SetP(frc::Preferences::GetDouble("kElevationP", c_defaultElevP));
  m_ElevationPIDController.SetI(frc::Preferences::GetDouble("kElevationI", c_defaultElevI));
  m_ElevationPIDController.SetD(frc::Preferences::GetDouble("kElevationD", c_defaultElevD));
  m_ElevationPIDController.SetFF(frc::Preferences::GetDouble("kElevationFF", c_defaultElevFF));
  m_ElevationPIDController.SetOutputRange(kMinOut, kMaxOut);
#endif

  frc::SmartDashboard::PutNumber("OverRPM",  m_shootReference[0][1]);
  frc::SmartDashboard::PutNumber("UnderRPM", m_shootReference[1][1]);
  frc::SmartDashboard::PutNumber("BackRPM", m_shootReference[2][1]);
  frc::SmartDashboard::PutNumber("OverRPMClose",  m_shootReference[0][0]);
  frc::SmartDashboard::PutNumber("UnderRPMClose", m_shootReference[1][0]);
  frc::SmartDashboard::PutNumber("BackRPMClose", m_shootReference[2][0]);
  frc::SmartDashboard::PutNumber("ElevationAngle", m_shootReference[3][1]);
  frc::SmartDashboard::PutNumber("ElevationAngleClose", m_shootReference[3][0]);
  frc::SmartDashboard::PutNumber("ElevationTurns", 0.0);
  frc::SmartDashboard::PutNumber("RelTurns", 0);
}

void ShooterSubsystem::Periodic()
{
  if (m_poppedPin == false)
  {
    HAL_ControlWord cw;
    if (HAL_GetControlWord(&cw) == 0)
    {
      if (cw.enabled)
      {
        printf("Popping the pin\n");
        GoToElevation(50_deg);
        m_poppedPin = true;
      }
    }
  }

  m_closeAngle = units::degree_t(frc::SmartDashboard::GetNumber("CloseAngle", 49.0));

  m_logOverRPM.Append(m_OverRelativeEnc.GetVelocity()); 
  m_logUnderRPM.Append(m_UnderRelativeEnc.GetVelocity());
#ifdef OVERUNDER
  m_logBackRPM.Append(m_BackRelativeEnc.GetVelocity());
  m_logCurrentAngle.Append(m_ElevationRelativeEnc.GetPosition());
  m_logCommandedAngle.Append(m_elevationAngle);
  m_logAbsoluteAngle.Append(m_ElevationEncoder.GetAbsolutePosition().GetValueAsDouble());
#endif

  static int count = 0;
  if (count++ % 20 == 0)
  { 
    frc::SmartDashboard::PutNumber("ShooterAngle", m_gyro.GetRoll().value());
  // -126 + 14x - 0.225x^2 EQUATION
    m_shootReference[0][1] = frc::SmartDashboard::GetNumber("OverRPM",  c_defaultRPM);
    m_shootReference[1][1] = frc::SmartDashboard::GetNumber("UnderRPM", c_defaultRPM);
    m_shootReference[2][1] = frc::SmartDashboard::GetNumber("BackRPM", c_defaultBackRPM);
    m_shootReference[0][0] = frc::SmartDashboard::GetNumber("OverRPMClose",  c_defaultRPM);
    m_shootReference[1][0] = frc::SmartDashboard::GetNumber("UnderRPMClose", c_defaultRPM);
    m_shootReference[2][0] = frc::SmartDashboard::GetNumber("BackRPMClose", c_defaultBackRPM);
    m_shootReference[3][1] = frc::SmartDashboard::GetNumber("ElevationAngle", 32.0);
    m_shootReference[3][0] = frc::SmartDashboard::GetNumber("ElevationAngleClose", 49.0);

#ifdef OVERUNDER
    m_ElevationPIDController.SetP(frc::Preferences::GetDouble("kElevationP", c_defaultElevP));
    m_ElevationPIDController.SetI(frc::Preferences::GetDouble("kElevationI", c_defaultElevI));
    m_ElevationPIDController.SetD(frc::Preferences::GetDouble("kElevationD", c_defaultElevD));
    m_ElevationPIDController.SetFF(frc::Preferences::GetDouble("kElevationFF", c_defaultElevFF));
#endif

    frc::SmartDashboard::PutNumber("OverRPM echo", m_OverRelativeEnc.GetVelocity());
    frc::SmartDashboard::PutNumber("UnderRPM echo", m_UnderRelativeEnc.GetVelocity());
#ifdef OVERUNDER
    frc::SmartDashboard::PutNumber("BackRPM echo", m_BackRelativeEnc.GetVelocity());
    frc::SmartDashboard::PutNumber("Elevation echo", m_ElevationRelativeEnc.GetPosition());
    frc::SmartDashboard::PutNumber("ElevABS echo", m_ElevationEncoder.GetAbsolutePosition().GetValueAsDouble());
#endif
  }
}

#ifdef OVERUNDER
void ShooterSubsystem::GoToElevation(units::degree_t angle)
{
  m_elevationAngle = angle.to<double>();
  printf("elevation angle %.3f", m_elevationAngle);
  // m_elevationAngle = frc::SmartDashboard::GetNumber("ElevationAngle", 44.0);

  if (angle < 25.0_deg)
  {
    m_elevationAngle = 25.0;
  }
  else if (angle > 60.0_deg)
  {
    m_elevationAngle = 60.0;
  }

  // Reference  Shoot Angle AbsEnc  RelEnc
  // Max        76          0.480   -0.775
  // Nut/Zero   50          0.020   -0.321
  // Start      44         -0.146   -0.162
  // Hump       24          0.430    0.252
  // Min         1         -0.056    0.740
  // Linear fit
  double turns = -0.0196 * m_elevationAngle + 0.677;
  // double turns = -0.0204 * m_elevationAngle + 0.742;  
  // turns = frc::SmartDashboard::GetNumber("RelTurns", 0);
  printf("elev angle %.3f turns %.3f\n", m_elevationAngle, turns);
  frc::SmartDashboard::PutNumber("ElevationTurns", turns);
  m_ElevationPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
  frc::SmartDashboard::PutNumber("ElevApplOut", m_ElevationController.GetAppliedOutput());
  frc::SmartDashboard::PutNumber("ElevBusV", m_ElevationController.GetBusVoltage());
  frc::SmartDashboard::PutNumber("ElevTemp", m_ElevationController.GetMotorTemperature());
  frc::SmartDashboard::PutNumber("ElevOutAmps", m_ElevationController.GetOutputCurrent());
}

void ShooterSubsystem::GoToElevation(int shootIndex)
{
  GoToElevation(units::degree_t(m_shootReference[3][shootIndex]));
}

void ShooterSubsystem::StartOverAndUnder(units::meter_t distance)
{
    m_shootIndex = distance < 2.0_m ? 0 : 1;

    m_overRPM = m_shootReference[0][m_shootIndex];
    m_underRPM = m_shootReference[1][m_shootIndex];
    m_backRPM = m_shootReference[2][m_shootIndex];
    printf("m_overRPM: %.3f m_backRPM: %.3f \n", m_overRPM, m_backRPM);

    double ffNeo = frc::Preferences::GetDouble("kShooterFF", c_defaultShootNeoFF);
    double ffVtx = frc::Preferences::GetDouble("kShootVortexFF", c_defaultShootVortexFF);
    m_OverPIDController.SetFF(ffNeo);
    m_UnderPIDController.SetFF(ffNeo);

    printf("over %.3f under %.3f back %.3f \n", m_overRPM, m_underRPM, m_backRPM);
    m_OverPIDController.SetReference(m_overRPM, rev::CANSparkBase::ControlType::kVelocity);
    m_UnderPIDController.SetReference(m_underRPM, rev::CANSparkBase::ControlType::kVelocity);
    //m_UnderPIDController.SetReference(m_underRPM, rev::CANSparkBase::ControlType::kVelocity);

    // m_BackPIDController.SetFF(0.0);
    // m_BackPIDController.SetReference(0.0, rev::CANSparkBase::ControlType::kVelocity);
    m_BackPIDController.SetFF(ffVtx);
    m_BackPIDController.SetReference(m_backRPM, rev::CANSparkBase::ControlType::kVelocity);
}
#endif

void ShooterSubsystem::Shoot(units::meter_t distance)
{
  //m_OverPIDController.SetIAccum(0);
  //m_UnderPIDController.SetIAccum(0);
  
  m_shootIndex = distance < 2.0_m ? 0 : 1;

  m_overRPM = m_shootReference[0][m_shootIndex];
  m_underRPM = m_shootReference[1][m_shootIndex];
  m_backRPM = m_shootReference[2][m_shootIndex];

  double ffNeo = frc::Preferences::GetDouble("kShooterFF", c_defaultShootNeoFF);
  double ffVtx = frc::Preferences::GetDouble("kShootVortexFF", c_defaultShootVortexFF);
  m_OverPIDController.SetFF(ffNeo);
  m_UnderPIDController.SetFF(ffNeo);

  printf("over %.3f under %.3f back %.3f \n", m_overRPM, m_underRPM, m_backRPM);
  m_OverPIDController.SetReference(m_overRPM, rev::CANSparkBase::ControlType::kVelocity);
  m_UnderPIDController.SetReference(m_underRPM, rev::CANSparkBase::ControlType::kVelocity);
  //m_UnderPIDController.SetReference(m_underRPM, rev::CANSparkBase::ControlType::kVelocity);
#ifdef OVERUNDER  
  m_BackPIDController.SetFF(ffVtx);
  //m_BackPIDController.SetReference(-m_overRPM, rev::CANSparkBase::ControlType::kVelocity);
  m_BackPIDController.SetReference(m_backRPM, rev::CANSparkBase::ControlType::kVelocity);
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
