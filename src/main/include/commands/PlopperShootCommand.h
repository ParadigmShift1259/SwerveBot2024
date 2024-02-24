#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class PlopperShootCommand : public frc2::CommandHelper<frc2::Command, PlopperShootCommand> 
{
public:
  explicit PlopperShootCommand(ISubsystemAccess& subsystemAccess);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  AmpSubsystem& m_amp;

  wpi::log::BooleanLogEntry m_logStartCommand;
};