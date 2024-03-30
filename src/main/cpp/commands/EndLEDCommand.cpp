#include "commands/EndLEDCommand.h"
#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

EndLEDCommand::EndLEDCommand(ISubsystemAccess& subsystemAccess)
    : m_led(subsystemAccess.GetLED())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetLED()});

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
}

bool EndLEDCommand::IsFinished()
{
  return (m_timer.HasElapsed(0.5_s));
}

void EndLEDCommand::End(bool interrupted)
{
  m_led.SetAnimation(m_led.GetDefaultColor(), LEDSubsystem::kSolid);
  m_led.SetCurrentAction(LEDSubsystem::CurrentAction::kIdle);
  m_logStartEndLEDCommand.Append(false);
}
