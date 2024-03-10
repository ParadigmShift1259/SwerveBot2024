#include "commands/ShootCommand.h"

#include <frc/Timer.h>

ShootCommand::ShootCommand(ISubsystemAccess& subsystemAccess, bool bIsAuto)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
    , m_intakeSubsystem(subsystemAccess.GetIntake())
    , m_bIsAuto(bIsAuto)
    , m_led(subsystemAccess.GetLED())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter(), &subsystemAccess.GetIntake(), &subsystemAccess.GetLED()});
	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartShootCommand = wpi::log::BooleanLogEntry(log, "/ShootCommand/startCommand");
}

void ShootCommand::Initialize()
{
  m_timer.Reset();
  m_timer.Start();
  m_logStartShootCommand.Append(true);
  // m_shooterSubsystem.Shoot(m_distance);
  m_intakeSubsystem.EjectNote();
}

void ShootCommand::Execute()
{
  if (!m_intakeSubsystem.IsNotePresent()) {
    m_led.SetAnimation(c_colorPink, LEDSubsystem::kStrobe);
  }
}

bool ShootCommand::IsFinished()
{
    return m_timer.HasElapsed(1.0_s);
//    return false;
}

void ShootCommand::End(bool interrupted)
{
  m_led.SetAnimation(m_led.GetDefaultColor(), LEDSubsystem::kSolid);
  m_led.SetRobotBusy(false);
  m_intakeSubsystem.SetTransferFinished(false);
  if (m_bIsAuto == false)
  {
    m_shooterSubsystem.Stop();
    m_shooterSubsystem.GoToElevation(33.0_deg);
  }
  m_intakeSubsystem.Stop();
  m_logStartShootCommand.Append(false);
}
