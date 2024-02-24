
#include "commands/PlopperShootCommand.h"

PlopperShootCommand::PlopperShootCommand(ISubsystemAccess& subsystemAccess)
 : m_amp(subsystemAccess.GetAmp())
{
  AddRequirements({&subsystemAccess.GetAmp()});
  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/ampShoot/startCommand");
}

void PlopperShootCommand::Initialize()
{
  m_logStartCommand.Append(true);
}

void PlopperShootCommand::Execute()
{
  m_amp.Set(0.5);
}

bool PlopperShootCommand::IsFinished()
{
  return false;
}

void PlopperShootCommand::End(bool interrupted) {
  m_amp.Stop();
  m_logStartCommand.Append(false);
}
