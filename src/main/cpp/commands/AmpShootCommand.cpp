#include "commands/AmpShootCommand.h"

AmpShootCommand::AmpShootCommand(ISubsystemAccess& subsystemAccess)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
    , m_intakeSubsystem(subsystemAccess.GetIntake())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter(), &subsystemAccess.GetIntake()});

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartAmpShootCommand = wpi::log::BooleanLogEntry(log, "/AmpShootCommand/startCommand");
}

void AmpShootCommand::Initialize()
{
  m_logStartAmpShootCommand.Append(true);
  m_intakeSubsystem.Set(-0.6);
  m_timer.Reset();
  m_timer.Start();
}

void AmpShootCommand::Execute()
{
}

bool AmpShootCommand::IsFinished()
{
    return m_timer.HasElapsed(0.5_s);
}

void AmpShootCommand::End(bool interrupted)
{
  m_intakeSubsystem.Stop();
  m_logStartAmpShootCommand.Append(false);
}
