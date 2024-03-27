#include "commands/PreShootCommand.h"
#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

PreShootCommand::PreShootCommand(ISubsystemAccess& subsystemAccess)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
    , m_led(subsystemAccess.GetLED())
    , m_vision(subsystemAccess.GetVision())
{
    AddRequirements(frc2::Requirements{
        &subsystemAccess.GetShooter()
      , &subsystemAccess.GetLED()
      , &subsystemAccess.GetVision()  
    });
    m_elevationAngle = m_shooterSubsystem.GetCloseAngle();
    
    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartPreShootCommand = wpi::log::BooleanLogEntry(log, "/PreShootCommand/startCommand");
}

void PreShootCommand::Initialize()
{
  m_distance = units::meter_t{m_vision.GetShotDistance()};
  frc::SmartDashboard::PutNumber("VisionDistance echo", m_distance.value());
  m_led.SetRobotBusy(true);
  int shootIndex = m_distance < 2.0_m ? 0 : 1;
  frc::SmartDashboard::PutNumber("ShootIndex", shootIndex);
  m_led.SetAnimation(c_colorPink, LEDSubsystem::kFlow);
  m_logStartPreShootCommand.Append(true);
  m_vision.EnableShooterLEDs();
  if (shootIndex == 0)
  {
    m_shooterSubsystem.GoToElevation(shootIndex);
  }
  else
  {
    units::degree_t angle = m_vision.GetShotAngle();
    if (angle.value() == 0.0)
    {
      m_shooterSubsystem.GoToElevation(shootIndex);
    }
    else
    {
      m_shooterSubsystem.GoToElevation(angle);
    }
  }
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
