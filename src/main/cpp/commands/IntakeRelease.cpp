
#include "commands/IntakeRelease.h"

IntakeRelease::IntakeRelease(ISubsystemAccess& subsystemAccess)
 : m_intake(subsystemAccess.GetIntake())
 , m_led(subsystemAccess.GetLED())
{
  AddRequirements({&subsystemAccess.GetIntake(), &subsystemAccess.GetLED()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeRelease/startCommand");
}

void IntakeRelease::Initialize()
{
  m_logStartCommand.Append(true);
}

void IntakeRelease::Execute()
{
  // m_intake.ExtendIntake();
  m_intake.Set(kReleaseSpeed);
}

bool IntakeRelease::IsFinished()
{
  return false;
}

void IntakeRelease::End(bool interrupted) {
  m_intake.Set(0);
  m_logStartCommand.Append(false);
}
