#include "commands/IntakeTransfer.h"

#include <frc2/command/WaitCommand.h>

#include <frc/SmartDashBoard/SmartDashboard.h>

constexpr double c_defaultIntakeSpeed = 0.25;

IntakeTransfer::IntakeTransfer(ISubsystemAccess& subsystemAccess) 
  :/*m_shooter(subsystemAccess.GetShooter())
  ,*/m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({/*&subsystemAccess.GetShooter(), */&subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeIngest/startCommand");
}

void IntakeTransfer::Initialize()
{
  m_logStartCommand.Append(true);
}

void IntakeTransfer::Execute()
{
  // m_shooter.GoToElevation(25_deg);
  // frc2::WaitCommand(0.25_s); // Wait for backplate to extend and turntable motor to engage
  auto speed = c_defaultIntakeSpeed;
  m_intake.Set(speed);
  //m_intake.Set(kIngestSpeed);
}

bool IntakeTransfer::IsFinished()
{
  printf("out of loop");
  if (!m_frontPassed)
  {
    printf("if");
    m_frontPassed = !m_intake.IsNotePresent();
  }
  else {
    printf("else");
    return m_intake.IsNotePresent();
  }
  return false;
}

void IntakeTransfer::End(bool interrupted) 
{
  // frc2::WaitCommand(2.5_s);
  m_intake.Set(0.0);
  m_logStartCommand.Append(false);
}