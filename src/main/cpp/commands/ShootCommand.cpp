#include "commands/ShootCommand.h"

#include <frc/Timer.h>

ShootCommand::ShootCommand(ISubsystemAccess& subsystemAccess)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
    , m_intakeSubsystem(subsystemAccess.GetIntake())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter(), &subsystemAccess.GetIntake()});
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

}

bool ShootCommand::IsFinished()
{
    return m_timer.HasElapsed(1.0_s);
//    return false;
}

void ShootCommand::End(bool interrupted)
{
  m_shooterSubsystem.Stop();
  m_intakeSubsystem.Stop();
  m_shooterSubsystem.GoToElevation(33.0_deg);
  m_logStartShootCommand.Append(false);
}
