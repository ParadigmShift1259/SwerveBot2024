#include "commands/StopAllCommand.h"

StopAllCommand::StopAllCommand(ISubsystemAccess& subsystemAccess)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
    , m_intakeSubsystem(subsystemAccess.GetIntake())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter(), &subsystemAccess.GetIntake()});

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/StopAllCommand/startCommand");
}

void StopAllCommand::Initialize()
{
  m_logStartCommand.Append(true);
  m_shooterSubsystem.Stop();
  m_intakeSubsystem.Stop();
}

void StopAllCommand::Execute()
{
}

bool StopAllCommand::IsFinished()
{
    return true;
}

void StopAllCommand::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}
