#include "commands/IntakeTransfer.h"

#include <frc2/command/WaitCommand.h>

#include <frc/SmartDashBoard/SmartDashboard.h>

constexpr double c_defaultIntakeSpeed = 0.25;

IntakeTransfer::IntakeTransfer(ISubsystemAccess& subsystemAccess) 
  : m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeTransfer/startCommand");
}

void IntakeTransfer::Initialize()
{
  m_logStartCommand.Append(true);
}

void IntakeTransfer::Execute()
{
    if (!m_intake.IsTransferFinished()) {
    auto speed = c_defaultIntakeSpeed;
    m_intake.Set(-speed);
  }
}

bool IntakeTransfer::IsFinished()
{
  if (m_intake.IsTransferFinished()) { return true; }
  if (!m_frontPassed)
  {
    m_frontPassed = !m_intake.IsNotePresent();
  }
  else {
    return m_intake.IsNotePresent();
  }
  return false;
}

void IntakeTransfer::End(bool interrupted) 
{
  m_intake.SetTransferFinished(true);
  m_intake.Set(0.0);
  m_logStartCommand.Append(false);
}