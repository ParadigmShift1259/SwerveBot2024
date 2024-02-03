
#include "commands/IntakeStop.h"

IntakeStop::IntakeStop(ISubsystemAccess& subsystemAccess)
 : m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeStop/startCommand");
}

void IntakeStop::Initialize()
{
  m_logStartCommand.Append(true);
}

void IntakeStop::Execute() {
  m_intake.Set(0);
}

bool IntakeStop::IsFinished()
{
  return true;
}

void IntakeStop::End(bool interrupted) {
  m_intake.RetractIntake();
  m_logStartCommand.Append(false);
}
