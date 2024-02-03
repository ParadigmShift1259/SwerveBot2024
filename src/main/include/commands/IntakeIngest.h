#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class IntakeIngest : public frc2::CommandHelper<frc2::Command, IntakeIngest>
{
public:
  explicit IntakeIngest(ISubsystemAccess& subsystemAccess);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem& m_intake;
  
  wpi::log::BooleanLogEntry m_logStartCommand;
};