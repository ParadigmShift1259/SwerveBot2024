#include "commands/IntakeAdjust.h"

IntakeAdjust::IntakeAdjust(ISubsystemAccess& subsystemAccess) 
  : m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeDeploy/startCommand");
}

void IntakeAdjust::Initialize()
{
  m_logStartCommand.Append(true);
}

void IntakeAdjust::Execute() 
{
  m_intake.AdjustIntake();
}

bool IntakeAdjust::IsFinished()
{
  return true;
}

void IntakeAdjust::End(bool interrupted) 
{
  m_logStartCommand.Append(false);
}