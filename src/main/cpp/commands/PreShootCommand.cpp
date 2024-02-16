#include "commands/PreShootCommand.h"
#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

PreShootCommand::PreShootCommand(ISubsystemAccess& subsystemAccess, units::meter_t distance, units::degree_t elevationAngle)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter()});
	
    m_distance = distance;
    m_elevationAngle = elevationAngle;

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartPreShootCommand = wpi::log::BooleanLogEntry(log, "/PreShootCommand/startCommand");
}

void PreShootCommand::Initialize()
{
  m_logStartPreShootCommand.Append(true);
  m_shooterSubsystem.GoToElevation(m_elevationAngle);
  //m_shooterSubsystem.GoToElevation(37.0_deg);
  m_shooterSubsystem.StartOverAndUnder(m_distance);
  printf("pre-shoot initialized \n");
}

void PreShootCommand::Execute()
{
}

bool PreShootCommand::IsFinished()
{
    return true;
}

void PreShootCommand::End(bool interrupted)
{
  m_logStartPreShootCommand.Append(false);
  printf("pre-shoot end \n");
}
