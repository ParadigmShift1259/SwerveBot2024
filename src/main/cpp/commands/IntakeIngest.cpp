#include "commands/IntakeIngest.h"

#include <frc2/command/WaitCommand.h>

#include <frc/SmartDashBoard/SmartDashboard.h>

constexpr double c_defaultIntakeSpeed = 0.95;

IntakeIngest::IntakeIngest(ISubsystemAccess& subsystemAccess) 
  : m_shooter(subsystemAccess.GetShooter())
  , m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetShooter(), &subsystemAccess.GetIntake()});

  frc::SmartDashboard::PutNumber("IntakeSpeed", c_defaultIntakeSpeed);

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeIngest/startCommand");
}

void IntakeIngest::Initialize()
{
  m_logStartCommand.Append(true);
}

void IntakeIngest::Execute()
{
  m_intake.ExtendIntake();
  m_shooter.GoToElevation(25_deg);
  frc2::WaitCommand(0.25_s); // Wait for backplate to extend and turntable motor to engage
  auto speed = frc::SmartDashboard::GetNumber("IntakeSpeed", c_defaultIntakeSpeed);
  m_intake.Set(speed);
  //m_intake.Set(kIngestSpeed);
}

bool IntakeIngest::IsFinished()
{
  return m_intake.IsNotePresent();
}

void IntakeIngest::End(bool interrupted) 
{
  // frc2::WaitCommand(2.5_s);
  m_intake.RetractIntake();
  m_intake.Set(0.0);
  m_logStartCommand.Append(false);
}