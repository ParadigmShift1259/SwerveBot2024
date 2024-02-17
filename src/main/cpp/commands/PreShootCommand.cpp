#include "commands/PreShootCommand.h"
#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

PreShootCommand::PreShootCommand(ISubsystemAccess& subsystemAccess, units::meter_t distance)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter()});
	
    m_distance = distance;
    m_elevationAngle = m_shooterSubsystem.GetCloseAngle();

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartPreShootCommand = wpi::log::BooleanLogEntry(log, "/PreShootCommand/startCommand");
}

void PreShootCommand::Initialize()
{
  int shootIndex = m_distance < 2.0_m ? 0 : 1;
  m_logStartPreShootCommand.Append(true);
  m_shooterSubsystem.GoToElevation(shootIndex);
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
