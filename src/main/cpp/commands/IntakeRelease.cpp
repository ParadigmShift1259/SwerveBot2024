
#include "commands/IntakeRelease.h"

constexpr double c_defaultIntakeReleaseSpeed = -1.0;

IntakeRelease::IntakeRelease(ISubsystemAccess& subsystemAccess)
 : m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeRelease/startCommand");
}

void IntakeRelease::Initialize()
{
  m_logStartCommand.Append(true);
}

void IntakeRelease::Execute()
{
  m_intake.Set(c_defaultIntakeReleaseSpeed);
}

bool IntakeRelease::IsFinished()
{
  return false;
}

void IntakeRelease::End(bool interrupted) {
  m_intake.Set(0);
  m_logStartCommand.Append(false);
}
