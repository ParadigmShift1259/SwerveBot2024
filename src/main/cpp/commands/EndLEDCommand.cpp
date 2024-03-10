#include "commands/EndLEDCommand.h"
#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

EndLEDCommand::EndLEDCommand(ISubsystemAccess& subsystemAccess)
    : m_intakeSubsystem(subsystemAccess.GetIntake())
    , m_led(subsystemAccess.GetLED())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetIntake(), &subsystemAccess.GetLED()});

    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartEndLEDCommand = wpi::log::BooleanLogEntry(log, "/EndLEDCommand/startCommand");
}

void EndLEDCommand::Initialize()
{
  m_timer.Reset();
  m_timer.Start();
  m_logStartEndLEDCommand.Append(true);
}

void EndLEDCommand::Execute()
{
  if (!m_intakeSubsystem.IsNotePresent()) {
    m_led.SetAnimation(c_colorPink, LEDSubsystem::kStrobe);
  }
}

bool EndLEDCommand::IsFinished()
{
  return (m_timer.HasElapsed(0.5_s));
}

void EndLEDCommand::End(bool interrupted)
{
  m_led.SetAnimation(m_led.GetDefaultColor(), LEDSubsystem::kSolid);
  m_led.SetRobotBusy(false);
  m_logStartEndLEDCommand.Append(false);
}
