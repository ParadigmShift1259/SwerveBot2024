#include "commands/IntakePreIngest.h"

#include <frc2/command/WaitCommand.h>

#include <frc/SmartDashBoard/SmartDashboard.h>

constexpr double c_defaultIntakeSpeed = 0.75;

IntakePreIngest::IntakePreIngest(ISubsystemAccess& subsystemAccess) 
  : m_shooter(subsystemAccess.GetShooter())
  , m_intake(subsystemAccess.GetIntake())
  , m_led(subsystemAccess.GetLED())
{
  AddRequirements({&subsystemAccess.GetShooter(), &subsystemAccess.GetIntake(), &subsystemAccess.GetLED()});

  frc::SmartDashboard::PutNumber("IntakeSpeed", c_defaultIntakeSpeed);
  frc::SmartDashboard::PutNumber("IntakeAngle", 48.0);

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/IntakePreIngest/startCommand");
}

void IntakePreIngest::Initialize()
{
  m_led.SetRobotBusy(true);
  m_logStartCommand.Append(true);
  m_led.SetAnimation(c_colorGreen, LEDSubsystem::kFlow);
}

void IntakePreIngest::Execute()
{
  m_intake.ExtendIntake();
  auto angle = frc::SmartDashboard::GetNumber("IntakeAngle", 48.0);
  m_shooter.GoToElevation(units::degree_t(angle));
}

bool IntakePreIngest::IsFinished()
{
  return true;
}

void IntakePreIngest::End(bool interrupted) 
{
  m_logStartCommand.Append(false);
}