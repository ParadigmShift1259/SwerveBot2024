#include "commands/ShootCommand.h"
#include <frc2/command/WaitCommand.h>

ShootCommand::ShootCommand(ISubsystemAccess& subsystemAccess)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter()});
	
	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartShootCommand = wpi::log::BooleanLogEntry(log, "/ShootCommand/startCommand");
}

void ShootCommand::Initialize()
{
  m_logStartShootCommand.Append(true);
  m_shooterSubsystem.GoToElevation(37.0_deg);
  m_shooterSubsystem.StartOverAndUnder();
  //frc2::WaitCommand(0.5_s);
  m_shooterSubsystem.Shoot(129_in);
}

void ShootCommand::Execute()
{
}

bool ShootCommand::IsFinished()
{
    return false;
}

void ShootCommand::End(bool interrupted)
{
  m_shooterSubsystem.Stop();
  m_logStartShootCommand.Append(false);
}
