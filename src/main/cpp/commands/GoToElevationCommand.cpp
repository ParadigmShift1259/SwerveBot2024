#include "commands/GoToElevationCommand.h"

GoToElevationCommand::GoToElevationCommand(ISubsystemAccess& subsystemAccess, units::degree_t angle)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter(), &subsystemAccess.GetIntake()});
	
    m_angle = angle;

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartGoToElevationCommand = wpi::log::BooleanLogEntry(log, "/GoToElevationCommand/startCommand");
}

void GoToElevationCommand::Initialize()
{
  m_logStartGoToElevationCommand.Append(true);
  m_shooterSubsystem.GoToElevation(m_angle);
}

void GoToElevationCommand::Execute()
{
}

bool GoToElevationCommand::IsFinished()
{
    return true;
}

void GoToElevationCommand::End(bool interrupted)
{
  m_logStartGoToElevationCommand.Append(false);
}
