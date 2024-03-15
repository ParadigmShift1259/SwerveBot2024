
#include "commands/AmpIntakeCommand.h"

#include <frc/SmartDashBoard/SmartDashboard.h>

AmpIntakeCommand::AmpIntakeCommand(ISubsystemAccess& subsystemAccess)
 : m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetIntake()});
  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/ampIntake/startCommand");
}

void AmpIntakeCommand::Initialize()
{
  m_logStartCommand.Append(true);
  double speed = frc::SmartDashboard::GetNumber("AmpIntakePercent", 0.5);
  printf("amp intake speed: %.3f\n", speed);
  m_intake.Set(speed);
}

void AmpIntakeCommand::Execute()
{
}

bool AmpIntakeCommand::IsFinished()
{
  return m_intake.IsNotePresent();
}

void AmpIntakeCommand::End(bool interrupted) {
  m_intake.Stop();
  m_logStartCommand.Append(false);
}
