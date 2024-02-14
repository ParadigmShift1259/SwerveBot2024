#include "commands/PreShootCommand.h"
#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

PreShootCommand::PreShootCommand(ISubsystemAccess& subsystemAccess)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter()});
	
	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartPreShootCommand = wpi::log::BooleanLogEntry(log, "/PreShootCommand/startCommand");
}

void PreShootCommand::Initialize()
{
  m_logStartPreShootCommand.Append(true);
  double elevationAngle = frc::SmartDashboard::GetNumber("ElevationAngle", 40.0);
  m_shooterSubsystem.GoToElevation(units::degree_t{elevationAngle});
  //m_shooterSubsystem.GoToElevation(37.0_deg);
  m_shooterSubsystem.StartOverAndUnder();
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
