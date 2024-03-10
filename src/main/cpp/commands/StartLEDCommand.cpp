#include "commands/StartLEDCommand.h"
#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

StartLEDCommand::StartLEDCommand(ISubsystemAccess& subsystemAccess)
    : m_led(subsystemAccess.GetLED())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetLED()});

    wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartStartLEDCommand = wpi::log::BooleanLogEntry(log, "/StartLEDCommand/startCommand");
}

void StartLEDCommand::Initialize()
{
  m_led.SetRobotBusy(true);
  m_led.SetAnimation(c_colorPink, LEDSubsystem::kFlow);
  m_logStartStartLEDCommand.Append(true);
}

void StartLEDCommand::Execute()
{
}

bool StartLEDCommand::IsFinished()
{
  return true;
}

void StartLEDCommand::End(bool interrupted)
{
  m_logStartStartLEDCommand.Append(false);
}
