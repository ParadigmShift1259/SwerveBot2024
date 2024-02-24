#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class AmpIntakeCommand : public frc2::CommandHelper<frc2::Command, AmpIntakeCommand> 
{
public:
  explicit AmpIntakeCommand(ISubsystemAccess& subsystemAccess);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  IntakeSubsystem& m_intake;

  wpi::log::BooleanLogEntry m_logStartCommand;
};