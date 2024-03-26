#include "commands/AmpShootCommand.h"

#include <frc/SmartDashBoard/SmartDashboard.h>

AmpShootCommand::AmpShootCommand(ISubsystemAccess& subsystemAccess)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
    , m_intakeSubsystem(subsystemAccess.GetIntake())
    , m_led(subsystemAccess.GetLED())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter(), &subsystemAccess.GetIntake(), &subsystemAccess.GetLED()});

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartAmpShootCommand = wpi::log::BooleanLogEntry(log, "/AmpShootCommand/startCommand");
}

void AmpShootCommand::Initialize()
{
  m_led.SetRobotBusy(true);
  m_logStartAmpShootCommand.Append(true);
  double speed = 0.6;//frc::SmartDashboard::GetNumber("AmpIntakePercent", 0.6);
  // printf("amp intake speed: %.3f\n", speed);
  m_intakeSubsystem.Set(-1.0 * speed);
  m_timer.Reset();
  m_timer.Start();
}

void AmpShootCommand::Execute()
{
  if (!m_intakeSubsystem.IsNotePresent())
  {
    m_led.SetAnimation(c_colorPink, LEDSubsystem::kStrobe);
  }
}

bool AmpShootCommand::IsFinished()
{
    return m_timer.HasElapsed(0.5_s);
}

void AmpShootCommand::End(bool interrupted)
{
  m_led.SetAnimation(m_led.GetDefaultColor(), LEDSubsystem::kSolid);
  m_led.SetRobotBusy(false);
  m_intakeSubsystem.SetTransferFinished(false);
  m_intakeSubsystem.Stop();
  m_logStartAmpShootCommand.Append(false);
}
