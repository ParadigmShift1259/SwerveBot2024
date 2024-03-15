#include "Constants.h"

#include "subsystems/ClimberSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

constexpr double c_defaultParkTurns = 0.0;
constexpr double c_defaultHighTurns = 170.0;

double c_defaultLeadDirection;
double c_defaultFollowDirection;

constexpr double c_defaultClimbP = 0.07;
constexpr double c_defaultClimbI = 0.0;
constexpr double c_defaultClimbD = 0.0;
constexpr double c_defaultClimbFF = 0.00000;

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

#ifdef THING1
ClimberSubsystem::ClimberSubsystem()
{
}

void ClimberSubsystem::Periodic()
{
}

void ClimberSubsystem::GoToPosition(double position)
{
}

void ClimberSubsystem::Set(double speed) 
{
}
#else
ClimberSubsystem::ClimberSubsystem()
    : m_leadMotor(kClimbLeadMotorCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
    , m_followMotor(kClimbFollowMotorCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
{
    m_leadMotor.RestoreFactoryDefaults();

    m_leadMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_leadMotor.SetInverted(false);
    m_leadMotor.SetClosedLoopRampRate(0.0);
    m_leadRelativeEnc.SetPosition(0.0);

    m_followMotor.RestoreFactoryDefaults();

    m_followMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_followMotor.SetInverted(true);
    m_followMotor.SetClosedLoopRampRate(0.0);
    m_followRelativeEnc.SetPosition(0.0);

    m_HighTurns = c_defaultHighTurns;
    m_ParkTurns = c_defaultParkTurns;

    c_defaultLeadDirection = m_leadDirection;
    c_defaultFollowDirection = m_followDirection;

    //m_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10.0, 1.0);
    //m_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10.0, 1.0);

    // m_motor.ConfigNeutralDeadband(0);
    // m_motor.ConfigNominalOutputForward(kNominal);
    // m_motor.ConfigNominalOutputReverse(kNominal * -1.0);
    // m_motor.ConfigPeakOutputForward(kMaxOut);
    // m_motor.ConfigPeakOutputReverse(kMaxOut * -1.0);
    frc::SmartDashboard::PutNumber("ClimbLeadMotorDirection", c_defaultLeadDirection);
    frc::SmartDashboard::PutNumber("ClimbFollowMotorDirection", c_defaultFollowDirection);

    frc::SmartDashboard::PutNumber("ClimbHiTurns", c_defaultHighTurns);
    frc::SmartDashboard::PutNumber("ClimbParkTurns", c_defaultParkTurns);

    frc::Preferences::InitDouble("kClimbPosP", c_defaultClimbP);
    frc::Preferences::InitDouble("kClimbPosI", c_defaultClimbI);
    frc::Preferences::InitDouble("kClimbPosD", c_defaultClimbD);
    frc::Preferences::InitDouble("kClimbPosFF", c_defaultClimbFF);

    frc::SmartDashboard::PutNumber("ClimbLeadMotorPos", 1.0);
    frc::SmartDashboard::PutNumber("ClimbFollowMotorPos", 1.0);

    m_leadPIDController.SetOutputRange(kMinOut, kMaxOut);
    m_followPIDController.SetOutputRange(kMinOut, kMaxOut);

}

void ClimberSubsystem::Periodic()
{
  static int count = 0;
  if (count++ % 20 == 0)
  {
    static double lastP = 0.0;
    static double lastI = 0.0;
    static double lastD = 0.0;
    static double lastFF = 0.0;

    auto p = frc::Preferences::GetDouble("kClimbPosP", c_defaultClimbP); //originally .07
    auto i = frc::Preferences::GetDouble("kClimbPosI", c_defaultClimbI);
    auto d = frc::Preferences::GetDouble("kClimbPosD", c_defaultClimbD);
    auto ff = frc::Preferences::GetDouble("kClimbPosFF", c_defaultClimbFF);
    if (p != lastP)
    {
        m_leadPIDController.SetP(p);
        m_followPIDController.SetP(p);
    }
    if (i != lastI)
    {
        m_leadPIDController.SetI(i);
        m_followPIDController.SetI(i);
    }
    if (d != lastD)
    {
        m_leadPIDController.SetD(d);
        m_followPIDController.SetD(d);
    }
    if (ff != lastFF)
    {
        m_leadPIDController.SetFF(ff);
       m_followPIDController.SetFF(ff);
    }
    lastP = p;
    lastI = i;
    lastD = d;
    lastFF = ff;

    frc::SmartDashboard::PutNumber("ClimbLeadMotorPos Echo", m_leadRelativeEnc.GetPosition());
    frc::SmartDashboard::PutNumber("ClimbFollowMotorPos Echo", m_followRelativeEnc.GetPosition());
  }
}

void ClimberSubsystem::GoToPosition(double position)
{
    m_leadPIDController.SetReference(position, rev::CANSparkBase::ControlType::kPosition);
    m_followPIDController.SetReference(position, rev::CANSparkBase::ControlType::kPosition);

}

void ClimberSubsystem::Set(double speed) 
{
    // m_motor.Set(ControlMode::PercentOutput, speed * ClimberConstants::kMotorReverseConstant);
    m_leadMotor.Set(speed);
    m_followMotor.Set(speed);
}
#endif