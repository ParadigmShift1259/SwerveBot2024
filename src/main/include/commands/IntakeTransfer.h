#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class IntakeTransfer : public frc2::CommandHelper<frc2::Command, IntakeTransfer>
{
public:
  explicit IntakeTransfer(ISubsystemAccess& subsystemAccess);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  // ShooterSubsystem& m_shooter;
  IntakeSubsystem& m_intake;
  bool m_frontPassed = false;
  
  wpi::log::BooleanLogEntry m_logStartCommand;
};