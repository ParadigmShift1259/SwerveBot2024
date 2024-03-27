#include "commands/StopShootCommand.h"

StopShootCommand::StopShootCommand(ISubsystemAccess& subsystemAccess)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
    // , m_intakeSubsystem(subsystemAccess.GetIntake())
    // , m_led(subsystemAccess.GetLED())
    // , m_drive(subsystemAccess.GetDrive())
    // , m_vision(subsystemAccess.GetVision())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter(), &subsystemAccess.GetIntake(), &subsystemAccess.GetLED(), &subsystemAccess.GetLED(), &subsystemAccess.GetVision()});

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/StopShootCommand/startCommand");
}

void StopShootCommand::Initialize()
{
  m_logStartCommand.Append(true);
  m_shooterSubsystem.Stop();
}

void StopShootCommand::Execute()
{
}

bool StopShootCommand::IsFinished()
{
  return true;
}

void StopShootCommand::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}
