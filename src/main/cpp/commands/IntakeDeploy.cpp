#include "commands/IntakeDeploy.h"

#include <frc/SmartDashBoard/SmartDashboard.h>

IntakeDeploy::IntakeDeploy(ISubsystemAccess& subsystemAccess) 
  : m_intake(subsystemAccess.GetIntake())
  , m_shooter(subsystemAccess.GetShooter())
{
  AddRequirements({&subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeDeploy/startCommand");
}

void IntakeDeploy::Initialize()
{
  m_logStartCommand.Append(true);
}

void IntakeDeploy::Execute() 
{
  m_intake.ExtendIntake();
  auto angle = frc::SmartDashboard::GetNumber("IntakeAngle", 42.0);
  m_shooter.GoToElevation(units::degree_t(angle));
}

bool IntakeDeploy::IsFinished()
{
  return true;
}

void IntakeDeploy::End(bool interrupted) 
{
  m_logStartCommand.Append(false);
}