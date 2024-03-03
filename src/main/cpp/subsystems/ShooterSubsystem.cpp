#include "ConstantsCANIDs.h"
#include "ConstantsDigitalInputs.h"

#include "subsystems/ShooterSubsystem.h"

#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <hal/HAL.h>

#include <wpi/deprecated.h>

constexpr double c_defaultRPM = 3700.0;

const units::degree_t c_pinPopAngle = 40_deg;
constexpr double c_elevStartAngle = 55.0;

constexpr double c_elevSlope = 1.09;
constexpr double c_elevOffset = -69.7;

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

const units::degree_t c_minElevAngle = -180.0_deg;
const units::degree_t c_maxElevAngle = 100.0_deg;

ShooterSubsystem::ShooterSubsystem()
  : m_elevationAngle(c_elevStartAngle)
  , m_gyro(kShooterPigeonCANID)
  , m_OverWheels(kShooterOverWheelsCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_UnderWheels(kShooterUnderWheelsCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_ElevationController(kShooterElevationControllerCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
  
{
  //m_OverWheels.RestoreFactoryDefaults();
  //m_UnderWheels.RestoreFactoryDefaults();
  //m_BackWheels.RestoreFactoryDefaults();

  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_logOverRPM = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/OverRPM");
  m_logUnderRPM = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/UnderRPM");
  m_logCurrentAngle = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/CurrentAngle");
  m_logCommandedAngle = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/CommandedAngle");
  m_logAbsoluteAngle = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/AbsoluteAngle");
  m_logElevTurns = wpi::log::DoubleLogEntry(log, "/subsystem/shooter/ElevTurns");

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

  // SmartMotion params
  frc::Preferences::InitDouble("kElevMinVel", 0.0);
  frc::Preferences::InitDouble("kElevMaxVel", 2000.0);
  frc::Preferences::InitDouble("kElevMaxAcc", 1500.0);
  frc::Preferences::InitDouble("kElevAllowedErr", 0.0);
  
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

  m_ElevationController.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_ElevationPIDController.SetP(frc::Preferences::GetDouble("kElevationP", c_defaultElevP));
  m_ElevationPIDController.SetI(frc::Preferences::GetDouble("kElevationI", c_defaultElevI));
  m_ElevationPIDController.SetD(frc::Preferences::GetDouble("kElevationD", c_defaultElevD));
  m_ElevationPIDController.SetFF(frc::Preferences::GetDouble("kElevationFF", c_defaultElevFF));
  m_ElevationPIDController.SetOutputRange(-1.0, 1.0);

  // Smart Motion Coefficients
  // double minVel = frc::Preferences::GetDouble("kElevMinVel", 0.0);     // rpm
  // double maxVel = frc::Preferences::GetDouble("kElevMaxVel", 2000.0);  // rpm
  // double maxAcc = frc::Preferences::GetDouble("kElevMaxAcc", 1500.0);
  // double allowedErr = frc::Preferences::GetDouble("kElevAllowedErr", 0.0);

  // int smartMotionSlot = 0;
  // m_ElevationPIDController.SetSmartMotionMaxVelocity(maxVel, smartMotionSlot);
  // m_ElevationPIDController.SetSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
  // m_ElevationPIDController.SetSmartMotionMaxAccel(maxAcc, smartMotionSlot);
//   // m_ElevationPIDController.SetSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);  
// WPI_IGNORE_DEPRECATED
//   m_ElevationPIDController.SetSmartMotionAccelStrategy(rev::SparkMaxPIDController::AccelStrategy::kSCurve, smartMotionSlot); // Other accel strategy kTrapezoidal
// WPI_UNIGNORE_DEPRECATED

  frc::SmartDashboard::PutNumber("ShotAngle", 37.0);
  frc::SmartDashboard::PutNumber("ShotAngleClose", 57.0);
  frc::SmartDashboard::PutNumber("OverRPM",  m_shootReference[0][1]);
  frc::SmartDashboard::PutNumber("UnderRPM", m_shootReference[0][1]);
  frc::SmartDashboard::PutNumber("ElevationAngle", 33.0);
  frc::SmartDashboard::PutNumber("ElevationTurns", 0.0);
  frc::SmartDashboard::PutBoolean("SetElevRef", false);

  auto pitch = m_gyro.GetPitch();
  double turns = (c_elevSlope * pitch + c_elevOffset);
  printf("gyro angle pitch %.3f gyro turns %.3f\n", pitch, turns);
  m_ElevationRelativeEnc.SetPosition(turns);
}

void ShooterSubsystem::Periodic()
{
  // if (m_poppedPin == false)
  // {
  //   HAL_ControlWord cw;
  //   if (HAL_GetControlWord(&cw) == 0)
  //   {
  //     if (cw.enabled)
  //     {
  //       printf("Popping the pin\n");
  //       GoToElevation(c_pinPopAngle);
  //       m_poppedPin = true;
  //     }
  //   }
  // }

  // if (m_lastElevationTurns != m_elevationTurns && fabs(m_ElevationRelativeEnc.GetPosition() - frc::SmartDashboard::GetNumber("ElevationTurns", 0.0)) < 1.0)
  // {
  //   m_lastElevationTurns = m_elevationTurns;
  //   // //double adj = pitch - c_elevStartAngle;
  //   // double turns = (1.13 * pitch - 69.5) / 1.75;
  //   auto pitch = m_gyro.GetPitch();
  //   printf("elev gyro angle %.3f rel enc %.3f turns %.3f\n", pitch, m_ElevationRelativeEnc.GetPosition(), m_elevationTurns);
  // }

  //m_closeAngle = units::degree_t(frc::SmartDashboard::GetNumber("CloseAngle", 49.0));

  m_logOverRPM.Append(m_OverRelativeEnc.GetVelocity()); 
  m_logUnderRPM.Append(m_UnderRelativeEnc.GetVelocity());
  m_logCurrentAngle.Append(m_ElevationRelativeEnc.GetPosition());
  m_logCommandedAngle.Append(m_elevationAngle);

  static int count = 0;
  if (count++ % 20 == 0)
  {
    auto pitch = m_gyro.GetPitch();
    auto ticks = m_ElevationRelativeEnc.GetPosition();
    double angle = (ticks - c_elevOffset) / c_elevSlope;
    frc::SmartDashboard::PutNumber("ElevationAngleEcho", angle);
    frc::SmartDashboard::PutNumber("ShooterAngle", pitch);
    // double turns = (c_elevSlope * pitch + c_elevOffset);
    // double adj = m_elevationTurns - turns;
    // printf("elev from gyro angle pitch %.3f elev angle %.3f adj %.3f goto angle %.3f gyro turns %.3f elev turns %.3f\n", pitch, angle, adj, angle + adj, turns, m_elevationTurns);
    // if (m_elevationTurns != 0.0 && fabs(adj) < 0.5)
    // {
    //   printf("Setting elev from gyro angle pitch %.3f elev angle %.3f adj %.3f goto angle %.3f gyro turns %.3f elev turns %.3f\n", pitch, angle, adj, angle + adj, turns, m_elevationTurns);
    //   angle = ((m_elevationTurns - adj) - c_elevOffset) / c_elevSlope;
    //   GoToElevation(units::degree_t(angle));
    //   m_elevationTurns = 0.0;
    // }

    m_shootReference[0][1] = frc::SmartDashboard::GetNumber("OverRPM",  -c_defaultRPM);
    m_shootReference[0][1] = frc::SmartDashboard::GetNumber("UnderRPM", c_defaultRPM);

    m_shootReference[0][0] = frc::SmartDashboard::GetNumber("OverRPMClose",  -c_defaultRPM);
    m_shootReference[0][0] = frc::SmartDashboard::GetNumber("UnderRPMClose", c_defaultRPM);

    m_shootReference[2][1] = frc::SmartDashboard::GetNumber("ShotAngle", 37.0);
    m_shootReference[2][0] = frc::SmartDashboard::GetNumber("ShotAngleClose", 57.0);

    static double lastP = 0.0;
    static double lastI = 0.0;
    static double lastD = 0.0;
    static double lastFF = 0.0;

    auto p = frc::Preferences::GetDouble("kElevationP", c_defaultElevP);
    auto i = frc::Preferences::GetDouble("kElevationI", c_defaultElevI);
    auto d = frc::Preferences::GetDouble("kElevationD", c_defaultElevD);
    auto ff = frc::Preferences::GetDouble("kElevationFF", c_defaultElevFF);
    if (p != lastP)
    {
        m_ElevationPIDController.SetP(p);
    }
    if (i != lastI)
    {
        m_ElevationPIDController.SetI(i);
    }
    if (d != lastD)
    {
        m_ElevationPIDController.SetD(d);
    }
    if (ff != lastFF)
    {
        m_ElevationPIDController.SetFF(ff);
    }
    lastP = p;
    lastI = i;
    lastD = d;
    lastFF = ff;

    static double lastMaxVel = 0.0;
    static double lastMaxAcc = 0.0;
    static double lastAllowedErr = 0.0;
    //double minVel = frc::Preferences::GetDouble("kElevMinVel", 0.0);     // rpm
    //m_ElevationPIDController.SetSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    double maxVel = frc::Preferences::GetDouble("kElevMaxVel", 2000.0);  // rpm
    double maxAcc = frc::Preferences::GetDouble("kElevMaxAcc", 1500.0);
    double allowedErr = frc::Preferences::GetDouble("kElevAllowedErr", 0.0);
    int smartMotionSlot = 0;
    if (maxVel != lastMaxVel)
    {
      m_ElevationPIDController.SetSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    }
    if (maxAcc != lastMaxAcc)
    {
      m_ElevationPIDController.SetSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    }
    if (allowedErr != lastAllowedErr)
    {
      m_ElevationPIDController.SetSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);  
    }
    lastMaxVel = maxVel;
    lastMaxAcc = maxAcc;
    lastAllowedErr = allowedErr;

    frc::SmartDashboard::PutNumber("OverRPM echo", m_OverRelativeEnc.GetVelocity());
    frc::SmartDashboard::PutNumber("UnderRPM echo", m_UnderRelativeEnc.GetVelocity());
    frc::SmartDashboard::PutNumber("Elevation echo", m_ElevationRelativeEnc.GetPosition());
  }
}

void ShooterSubsystem::GoToElevation(units::degree_t angle)
{
  m_elevationAngle = angle.to<double>();
  //m_elevationAngle = frc::SmartDashboard::GetNumber("ElevationAngle", 44.0);

  if (angle < c_minElevAngle)
  {
    m_elevationAngle = c_minElevAngle.value();
  }
  else if (angle > c_maxElevAngle)
  {
    m_elevationAngle = c_maxElevAngle.value();
  }

  // Shoot  Rel
  // Angle  Enc
  // 65.9   0
  // 60.4   -6.214
  // 52.1   -15.452
  // 43.3   -25.356
  // 32.8   -37.285
  // 22.9   -47.856
  // 11.2   -60.858
  // 0.2    -74.359
//#define DASHBOARD_OVERRIDE
#ifdef DASHBOARD_OVERRIDE
  double turns = frc::SmartDashboard::GetNumber("ElevationTurns", 0.0);
#else
  double turns = (c_elevSlope * m_elevationAngle + c_elevOffset);
  frc::SmartDashboard::PutNumber("ElevationTurns", turns);
#endif

  //("elev angle %.3f turns %.3f\n", m_elevationAngle, turns);
  m_elevationTurns = turns;  // For calibration
  m_logElevTurns.Append(turns);

  //bool setRef = frc::SmartDashboard::GetBoolean("SetElevRef", false);
  // if (setRef)
  // {
     m_ElevationPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
  // }
  // m_ElevationPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kSmartMotion);

  // frc::SmartDashboard::PutNumber("ElevApplOut", m_ElevationController.GetAppliedOutput());
  // frc::SmartDashboard::PutNumber("ElevBusV", m_ElevationController.GetBusVoltage());
  // frc::SmartDashboard::PutNumber("ElevTemp", m_ElevationController.GetMotorTemperature());
  // frc::SmartDashboard::PutNumber("ElevOutAmps", m_ElevationController.GetOutputCurrent());
}

void ShooterSubsystem::GoToElevation(int shootIndex)
{
  GoToElevation(units::degree_t(m_shootReference[2][shootIndex]));
}

void ShooterSubsystem::StartOverAndUnder(units::meter_t distance)
{
    m_shootIndex = distance < 2.0_m ? 0 : 1;

    m_overRPM = -m_shootReference[0][m_shootIndex];
    m_underRPM = m_shootReference[0][m_shootIndex];

    double ffNeo = frc::Preferences::GetDouble("kShooterFF", c_defaultShootNeoFF);
    m_OverPIDController.SetFF(ffNeo);
    m_UnderPIDController.SetFF(ffNeo);

    printf("over %.3f under %.3f\n", m_overRPM, m_underRPM);
    m_OverPIDController.SetReference(m_overRPM, rev::CANSparkBase::ControlType::kVelocity);
    m_UnderPIDController.SetReference(m_underRPM, rev::CANSparkBase::ControlType::kVelocity);
}

void ShooterSubsystem::Shoot(units::meter_t distance)
{
  //m_OverPIDController.SetIAccum(0);
  //m_UnderPIDController.SetIAccum(0);
  
  m_shootIndex = distance < 2.0_m ? 0 : 1;

  m_overRPM = -m_shootReference[0][m_shootIndex];
  m_underRPM = m_shootReference[0][m_shootIndex];

  double ffNeo = frc::Preferences::GetDouble("kShooterFF", c_defaultShootNeoFF);
  m_OverPIDController.SetFF(ffNeo);
  m_UnderPIDController.SetFF(ffNeo);

  printf("over %.3f under %.3f\n", m_overRPM, m_underRPM);
  m_OverPIDController.SetReference(m_overRPM, rev::CANSparkBase::ControlType::kVelocity);
  m_UnderPIDController.SetReference(m_underRPM, rev::CANSparkBase::ControlType::kVelocity);
}

void ShooterSubsystem::Stop()
{
  m_OverWheels.StopMotor();
  m_UnderWheels.StopMotor();
}
