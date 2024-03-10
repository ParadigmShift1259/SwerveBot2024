#include "subsystems/ClimberSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

double c_defaultParkTurns;
double c_defaultHighTurns;

double c_defaultLeadDirection;
double c_defaultFollowDirection;

constexpr double c_defaultClimbP = 0.07;
constexpr double c_defaultClimbI = 0.0;
constexpr double c_defaultClimbD = 0.0;

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

ClimberSubsystem::ClimberSubsystem() // 
    : m_leadMotor(kClimbLeadMotorCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
    , m_followMotor(kClimbFollowMotorCANID, rev::CANSparkLowLevel::MotorType::kBrushless)
{
    // m_leadMotor.RestoreFactoryDefaults();

    m_leadMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_leadMotor.SetInverted(false);
    m_leadMotor.SetClosedLoopRampRate(0.0);
    m_leadRelativeEnc.SetPosition(0.0);

    // m_followMotor.RestoreFactoryDefaults();

    m_followMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_followMotor.SetInverted(true);
    m_followMotor.SetClosedLoopRampRate(0.0);
    m_followRelativeEnc.SetPosition(0.0);

    c_defaultHighTurns = m_HighTurns;
    c_defaultParkTurns = m_ParkTurns;

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
    m_leadPIDController.SetP(frc::Preferences::GetDouble("kClimbPosP", c_defaultClimbP));
    m_leadPIDController.SetI(frc::Preferences::GetDouble("kClimbPosI", c_defaultClimbI));
    m_leadPIDController.SetD(frc::Preferences::GetDouble("kClimbPosD", c_defaultClimbD));
    
    m_followPIDController.SetP(frc::Preferences::GetDouble("kClimbPosP", c_defaultClimbP));
    m_followPIDController.SetI(frc::Preferences::GetDouble("kClimbPosI", c_defaultClimbI));
    m_followPIDController.SetD(frc::Preferences::GetDouble("kClimbPosD", c_defaultClimbD));

    m_leadDirection = frc::SmartDashboard::GetNumber("ClimbLeadMotorDirection", m_leadDirection);
    m_followDirection = frc::SmartDashboard::GetNumber("ClimbFollowMotorDirection", m_followDirection);

    m_HighTurns = frc::SmartDashboard::GetNumber("ClimbHiTurns", c_defaultHighTurns);
    m_ParkTurns     = frc::SmartDashboard::GetNumber("ClimbParkTurns", c_defaultParkTurns);

    frc::SmartDashboard::PutNumber("ClimbLeadMotorPos Echo", m_leadRelativeEnc.GetPosition());
    frc::SmartDashboard::PutNumber("ClimbFollowMotorPos Echo", m_followRelativeEnc.GetPosition());

    m_climbLeadPosition = frc::SmartDashboard::GetNumber("ClimbLeadMotorPos", 1.0);
    //m_climbFollowPosition = frc::SmartDashboard::GetNumber("ClimbFollowMotorPos", 1.0);
    m_climbFollowPosition = frc::SmartDashboard::GetNumber("ClimbLeadMotorPos", 1.0);
  }
}

void ClimberSubsystem::GoToPosition(double position)
{
    double leadTurns = m_climbLeadPosition;
    double followTurns = m_climbFollowPosition;
    printf("plopper lead turns %.3f\n", leadTurns);
    printf("plopper follow turns %.3f\n", followTurns);
    m_leadPIDController.SetReference(leadTurns, rev::CANSparkBase::ControlType::kPosition);
    m_followPIDController.SetReference(followTurns, rev::CANSparkBase::ControlType::kPosition);

}

void ClimberSubsystem::Set(double speed) 
{
    // m_motor.Set(ControlMode::PercentOutput, speed * ClimberConstants::kMotorReverseConstant);
    m_leadMotor.Set(speed * m_leadDirection);
    m_followMotor.Set(speed * m_followDirection);
}