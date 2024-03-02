
#include "ConstantsDigitalInputs.h"
#include "subsystems/IntakeSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

constexpr double c_defaultIntakeP = 0.03;//0.07;
constexpr double c_defaultIntakeI = 0.0;
constexpr double c_defaultIntakeD = 0.0;

constexpr double c_defaultIntakeMin = -0.5;
constexpr double c_defaultIntakeMax = 0.5;

constexpr double c_defaultRetractTurns = 0.0;
constexpr double c_defaultExtendTurns = 45.0;

using namespace frc;
using namespace ctre::phoenix::motorcontrol;

IntakeSubsystem::IntakeSubsystem() 
    : m_motor(kIntakeRollerCANID)
    , m_photoEye(kIntakePhotoeye)
    , m_deployMotor(kIntakeDeployCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
    , m_deployFollowMotor(kIntakeDeployFollowCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
{
    m_motor.SetNeutralMode(NeutralMode::Coast);

    m_deployMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_deployMotor.SetClosedLoopRampRate(0.0);
    m_deployRelativeEnc.SetPosition(0.0);

    frc::Preferences::InitDouble("kIntakeDeployP", c_defaultIntakeP);
    frc::Preferences::InitDouble("kIntakeDeployI", c_defaultIntakeI);
    frc::Preferences::InitDouble("kIntakeDeployD", c_defaultIntakeD);

    frc::Preferences::InitDouble("kIntakeDeployMin", c_defaultIntakeMin);
    frc::Preferences::InitDouble("kIntakeDeployMax", c_defaultIntakeMax);

#ifdef USE_SMART_MOTION_DEPLOY
    frc::Preferences::InitDouble("kDeployMinVel", 0.0);     // rpm
    frc::Preferences::InitDouble("kDeployMaxVel", 2000.0);  // rpm
    frc::Preferences::InitDouble("kDeployMaxAcc", 1500.0);
    frc::Preferences::InitDouble("kDeployAllowedErr", 0.0);
#endif

    //m_deployPIDController.SetOutputRange(kMinOut, kMaxOut);
    m_deployPIDController.SetOutputRange(-0.5, 0.5);

    frc::SmartDashboard::PutNumber("DepRtctTurns", c_defaultRetractTurns);
    frc::SmartDashboard::PutNumber("DepExtTurns", c_defaultExtendTurns);

    m_deployFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_deployFollowMotor.SetClosedLoopRampRate(0.0);
    m_deployFollowMotor.SetInverted(true);
    m_deployFollowRelativeEnc.SetPosition(0.0);
    //m_deployFollowPIDController.SetOutputRange(kMinOut, kMaxOut);
    m_deployFollowPIDController.SetOutputRange(-0.5, 0.5);
    // m_deployFollowMotor.Follow(m_deployMotor, true);

#ifdef USE_SMART_MOTION_DEPLOY
  // Smart Motion Coefficients
  double minVel = frc::Preferences::GetDouble("kDeployMinVel", 0.0);     // rpm
  double maxVel = frc::Preferences::GetDouble("kDeployMaxVel", 2000.0);  // rpm
  double maxAcc = frc::Preferences::GetDouble("kDeployMaxAcc", 1500.0);
  double allowedErr = frc::Preferences::GetDouble("kDeployAllowedErr", 0.0);

  int smartMotionSlot = 0;
  m_deployPIDController.SetSmartMotionMaxVelocity(maxVel, smartMotionSlot);
  m_deployPIDController.SetSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
  m_deployPIDController.SetSmartMotionMaxAccel(maxAcc, smartMotionSlot);
  m_deployPIDController.SetSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);  
  //m_deployPIDController.SetSmartMotionAccelStrategy(rev::SparkMaxPIDController::AccelStrategy::kSCurve, smartMotionSlot); // Other accel strategy kTrapezoidal
  m_deployPIDController.SetSmartMotionAccelStrategy(rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal, smartMotionSlot); // Other accel strategy kTrapezoidal

  m_deployFollowPIDController.SetSmartMotionMaxVelocity(maxVel, smartMotionSlot);
  m_deployFollowPIDController.SetSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
  m_deployFollowPIDController.SetSmartMotionMaxAccel(maxAcc, smartMotionSlot);
  m_deployFollowPIDController.SetSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);  
  //m_deployFollowPIDController.SetSmartMotionAccelStrategy(rev::SparkMaxPIDController::AccelStrategy::kSCurve, smartMotionSlot); // Other accel strategy kTrapezoidal
  m_deployFollowPIDController.SetSmartMotionAccelStrategy(rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal, smartMotionSlot); // Other accel strategy kTrapezoidal
#endif
}

void IntakeSubsystem::Periodic()
{
    LoadDeployPid();
#ifdef USE_SMART_MOTION_DEPLOY
    LoadDeploySmartmotion();
#endif

    frc::SmartDashboard::PutNumber("Deploy echo", m_deployRelativeEnc.GetPosition());
    frc::SmartDashboard::PutNumber("Deploy Follow echo", m_deployFollowRelativeEnc.GetPosition());
    frc::SmartDashboard::PutBoolean("Intake PhotoEye", m_photoEye.Get());
}

void IntakeSubsystem::LoadDeployPid()
{
    static double lastP = 0.0;
    static double lastI = 0.0;
    static double lastD = 0.0;

    auto p = frc::Preferences::GetDouble("kIntakeDeployP", c_defaultIntakeP);
    auto i = frc::Preferences::GetDouble("kIntakeDeployI", c_defaultIntakeI);
    auto d = frc::Preferences::GetDouble("kIntakeDeployD", c_defaultIntakeD);
    if (p != lastP)
    {
        m_deployPIDController.SetP(p);
        m_deployFollowPIDController.SetP(frc::Preferences::GetDouble("kIntakeDeployP", c_defaultIntakeP));
    }
    if (i != lastI)
    {
        m_deployPIDController.SetI(i);
        m_deployFollowPIDController.SetI(frc::Preferences::GetDouble("kIntakeDeployI", c_defaultIntakeI));
    }
    if (d != lastD)
    {
        m_deployPIDController.SetD(d);
        m_deployFollowPIDController.SetD(frc::Preferences::GetDouble("kIntakeDeployD", c_defaultIntakeD));
    }
    lastP = p;
    lastI = i;
    lastD = d;

    static double lastMin = 0.0;
    static double lastMax = 0.0;
    auto min = frc::Preferences::GetDouble("kIntakeDeployMin", c_defaultIntakeMin);
    auto max = frc::Preferences::GetDouble("kIntakeDeployMax", c_defaultIntakeMax);
    if (min != lastMin || max != lastMax)
    {
        m_deployPIDController.SetOutputRange(min, max);
        m_deployFollowPIDController.SetOutputRange(min, max);
    }
    lastMin = min;
    lastMax = max;
}

void IntakeSubsystem::LoadDeploySmartmotion()
{
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
      m_deployPIDController.SetSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      m_deployFollowPIDController.SetSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    }
    if (maxAcc != lastMaxAcc)
    {
      m_deployPIDController.SetSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      m_deployFollowPIDController.SetSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    }
    if (allowedErr != lastAllowedErr)
    {
      m_deployPIDController.SetSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);  
      m_deployFollowPIDController.SetSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);  
    }
    lastMaxVel = maxVel;
    lastMaxAcc = maxAcc;
    lastAllowedErr = allowedErr;
}

void IntakeSubsystem::Set(double speed)
{
    m_motor.Set(ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::ExtendIntake()
{
    double turns = frc::SmartDashboard::GetNumber("DepExtTurns", c_defaultExtendTurns);
    //printf("dep turns %.3f\n", turns);
#ifdef USE_SMART_MOTION_DEPLOY
    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kSmartMotion);
#else
    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
#endif

#ifdef USE_SMART_MOTION_DEPLOY
    m_deployFollowPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kSmartMotion);
#else
    m_deployFollowPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
#endif
    // frc::SmartDashboard::PutNumber("DepApplOut", m_deployMotor.GetAppliedOutput()); 
    // frc::SmartDashboard::PutNumber("DepBusV", m_deployMotor.GetBusVoltage());
    // frc::SmartDashboard::PutNumber("DepTemp", m_deployMotor.GetMotorTemperature());
    // frc::SmartDashboard::PutNumber("DepOutAmps", m_deployMotor.GetOutputCurrent());    
}

void IntakeSubsystem::ExtendIntake(double turns)
{
    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
    m_deployFollowPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
}

void IntakeSubsystem::AdjustIntake()
{
#ifdef OVERUNDER
    double turns = frc::SmartDashboard::GetNumber("DepAdjTurns", c_defaultAdjustTurns);
    //printf("dep turns %.3f\n", turns);
    //m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kSmartMotion);
    frc::SmartDashboard::PutNumber("DepApplOut", m_deployMotor.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("DepBusV", m_deployMotor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("DepTemp", m_deployMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("DepOutAmps", m_deployMotor.GetOutputCurrent());    
#endif
}

void IntakeSubsystem::RetractIntake()
{
    double turns = frc::SmartDashboard::GetNumber("DepRtctTurns", c_defaultRetractTurns);
    //printf("dep turns %.3f\n", turns);

    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);

    m_deployFollowPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
}

void IntakeSubsystem::GoToPosition(double turns)
{
    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);

    m_deployFollowPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
    
}
