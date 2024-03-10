#include "commands/PreShootCommand.h"
#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

PreShootCommand::PreShootCommand(ISubsystemAccess& subsystemAccess, units::meter_t distance)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
    , m_led(subsystemAccess.GetLED())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter(), &subsystemAccess.GetLED()});

	
    m_distance = distance;
    m_elevationAngle = m_shooterSubsystem.GetCloseAngle();
    
    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartPreShootCommand = wpi::log::BooleanLogEntry(log, "/PreShootCommand/startCommand");
}

void PreShootCommand::Initialize()
{
  m_led.SetRobotBusy(true);
  int shootIndex = m_distance < 2.0_m ? 0 : 1;
  m_led.SetAnimation(c_colorPink, LEDSubsystem::kFlow);
  m_logStartPreShootCommand.Append(true);
  m_shooterSubsystem.GoToElevation(shootIndex);
  //double elevationAngle = frc::SmartDashboard::GetNumber("ShotAngle", 39.0);
  //m_shooterSubsystem.GoToElevation(units::degree_t{elevationAngle});
  //m_shooterSubsystem.GoToElevation(39.0_deg);
  m_shooterSubsystem.StartOverAndUnder(m_distance);
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
}
