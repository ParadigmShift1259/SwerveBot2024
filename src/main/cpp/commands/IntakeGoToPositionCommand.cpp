#include "commands/IntakeGoToPositionCommand.h"

IntakeGoToPositionCommand::IntakeGoToPositionCommand(ISubsystemAccess& subsystemAccess, double turns)
    : m_intakeSubsystem(subsystemAccess.GetIntake())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetIntake()});
	
    m_turns = turns;

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartIntakeGoToPositionCommand = wpi::log::BooleanLogEntry(log, "/IntakeGoToPositionCommand/startCommand");
}

void IntakeGoToPositionCommand::Initialize()
{
  m_logStartIntakeGoToPositionCommand.Append(true);
  m_intakeSubsystem.GoToPosition(m_turns);
}

void IntakeGoToPositionCommand::Execute()
{
}

bool IntakeGoToPositionCommand::IsFinished()
{
    return true;
}

void IntakeGoToPositionCommand::End(bool interrupted)
{
  m_logStartIntakeGoToPositionCommand.Append(false);
}
