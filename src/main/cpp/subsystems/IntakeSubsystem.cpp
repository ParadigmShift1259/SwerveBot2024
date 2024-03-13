
#include "ConstantsDigitalInputs.h"
#include "subsystems/IntakeSubsystem.h"

#include <frc/SmartDashBoard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/Preferences.h>

constexpr double c_defaultIntakeP = 0.03;
constexpr double c_defaultIntakeI = 0.0;
constexpr double c_defaultIntakeD = 0.0;

constexpr double c_defaultIntakeMin = -0.5;
constexpr double c_defaultIntakeMax = 0.5;

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

    m_deployPIDController.SetOutputRange(kDeployMinOut, kDeployMaxOut);

    frc::SmartDashboard::PutNumber("DepRtctTurns", c_defaultRetractTurns);
    frc::SmartDashboard::PutNumber("DepExtTurns", c_defaultExtendTurns);

    m_deployFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_deployFollowMotor.SetClosedLoopRampRate(0.0);
    m_deployFollowMotor.SetInverted(true);
    m_deployFollowRelativeEnc.SetPosition(0.0);
    m_deployFollowPIDController.SetOutputRange(kDeployMinOut, kDeployMaxOut);
}

void IntakeSubsystem::Periodic()
{
    LoadDeployPid();

    frc::SmartDashboard::PutNumber("Deploy echo", m_deployRelativeEnc.GetPosition());
    frc::SmartDashboard::PutNumber("Deploy Follow echo", m_deployFollowRelativeEnc.GetPosition());
    frc::SmartDashboard::PutBoolean("Intake PhotoEye", m_photoEye.Get());
    frc::SmartDashboard::PutBoolean("Transfer Complete", m_transferComplete);
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

void IntakeSubsystem::Set(double speed)
{
    m_motor.Set(ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::ExtendIntake()
{
    double turns = frc::SmartDashboard::GetNumber("DepExtTurns", c_defaultExtendTurns);
    //printf("dep extend turns %.3f\n", turns);
    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);

    m_deployFollowPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
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


void IntakeSubsystem::RetractIntake()
{
    double turns = frc::SmartDashboard::GetNumber("DepRtctTurns", c_defaultRetractTurns);
    //printf("dep retract turns %.3f\n", turns);

    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
    m_deployFollowPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
}

void IntakeSubsystem::GoToPosition(double turns)
{
    m_deployPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
    m_deployFollowPIDController.SetReference(turns, rev::CANSparkBase::ControlType::kPosition);
}
