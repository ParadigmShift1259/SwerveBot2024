#include "commands/KillEmAllCommand.h"

KillEmAllCommand::KillEmAllCommand(ISubsystemAccess& subsystemAccess)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
    , m_led(subsystemAccess.GetLED())
    , m_drive(subsystemAccess.GetDrive())
    , m_vision(subsystemAccess.GetVision())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter(), &subsystemAccess.GetLED(), &subsystemAccess.GetLED(), &subsystemAccess.GetVision()});

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/KillEmAllCommand/startCommand");
}

void KillEmAllCommand::Initialize()
{
  m_logStartCommand.Append(true);
  m_shooterSubsystem.Stop();
  m_drive.Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false);
  m_led.SetAnimation(m_led.GetDefaultColor(), LEDSubsystem::kSolid);
  if ((m_led.GetCurrentAction() != LEDSubsystem::CurrentAction::kAmpMovement)
  && (m_led.GetCurrentAction() != LEDSubsystem::CurrentAction::kAmpPosition)
  && (m_led.GetCurrentAction() != LEDSubsystem::CurrentAction::kAmpShoot))
  {
    m_led.SetCurrentAction(LEDSubsystem::CurrentAction::kIdle);
  }
  m_vision.DisableShooterLEDs();
}

void KillEmAllCommand::Execute()
{
}

bool KillEmAllCommand::IsFinished()
{
  return true;
}

void KillEmAllCommand::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}
