#include "commands/IntakeIngest.h"

#include <frc2/command/WaitCommand.h>

IntakeIngest::IntakeIngest(ISubsystemAccess& subsystemAccess) 
  : m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeIngest/startCommand");
}

void IntakeIngest::Initialize()
{
  m_logStartCommand.Append(true);
}

void IntakeIngest::Execute()
{
  m_intake.ExtendIntake();
  frc2::WaitCommand(0.25_s); // Wait for backplate to extend and turntable motor to engage
  // m_intake.Set(kIngestSpeed);
}

void IntakeIngest::End(bool interrupted) 
{
  m_logStartCommand.Append(false);
}