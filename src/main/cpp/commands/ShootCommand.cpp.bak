#include "commands/ShootCommand.h"

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
  m_logStartShootCommand.Append(true);
  // m_shooterSubsystem.Shoot(m_distance);
  m_intakeSubsystem.EjectNote();
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
  m_intakeSubsystem.Stop();
  m_shooterSubsystem.GoToElevation(40.0_deg);
  m_logStartShootCommand.Append(false);
}
