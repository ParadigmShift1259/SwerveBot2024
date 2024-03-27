#include "commands/IntakeIngest.h"

#include <frc2/command/WaitCommand.h>

#include <frc/SmartDashBoard/SmartDashboard.h>

constexpr double c_defaultIntakeSpeed = 0.75;

IntakeIngest::IntakeIngest(ISubsystemAccess& subsystemAccess) 
  : m_shooter(subsystemAccess.GetShooter())
  , m_intake(subsystemAccess.GetIntake())
  , m_led(subsystemAccess.GetLED())
{
  AddRequirements({&subsystemAccess.GetShooter(), &subsystemAccess.GetIntake(), &subsystemAccess.GetLED()});

  frc::SmartDashboard::PutNumber("IntakeSpeed", c_defaultIntakeSpeed);
  frc::SmartDashboard::PutNumber("IntakeAngle", 48.0);
  frc::SmartDashboard::PutNumber("IntakeFinalAngle", 55.0);

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeIngest/startCommand");
}

void IntakeIngest::Initialize()
{
  m_led.SetRobotBusy(true);
  m_logStartCommand.Append(true);
  m_led.SetAnimation(c_colorGreen, LEDSubsystem::kFlow);
}

void IntakeIngest::Execute()
{
  if (m_intake.GetPosition() < 10.0)
  {
    m_intake.ExtendIntake();
    auto angle = frc::SmartDashboard::GetNumber("IntakeAngle", 48.0);
    m_shooter.GoToElevation(units::degree_t(angle));
    auto speed = frc::SmartDashboard::GetNumber("IntakeSpeed", c_defaultIntakeSpeed);
    m_intake.Set(speed);
  }
  else if (m_intake.GetPosition() > 35.0)
  {
    auto angle = frc::SmartDashboard::GetNumber("IntakeFinalAngle", 55.0);
    m_shooter.GoToElevation(units::degree_t(angle));
  }

  //frc2::WaitCommand(0.25_s); // Wait for backplate to extend and turntable motor to engage
}

bool IntakeIngest::IsFinished()
{
  return m_intake.IsNotePresent();
}

void IntakeIngest::End(bool interrupted) 
{
  m_led.SetRobotBusy(false);
  if (!interrupted) {
    m_led.SetDefaultColor(c_colorPink);
    m_led.SetAnimation(c_colorPink, LEDSubsystem::kSolid);
  }
  m_intake.RetractIntake();
  m_intake.Set(0.0);
  auto angle = frc::SmartDashboard::GetNumber("Travel Angle", c_defaultTravelPosition.value());
  m_shooter.GoToElevation(units::degree_t(angle));
  m_shooter.StartOverAndUnder(-1.0_m);
  m_logStartCommand.Append(false);
}