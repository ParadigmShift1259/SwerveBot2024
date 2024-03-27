#include "commands/StopAllCommand.h"

StopAllCommand::StopAllCommand(ISubsystemAccess& subsystemAccess)
    : m_shooterSubsystem(subsystemAccess.GetShooter())
    , m_intakeSubsystem(subsystemAccess.GetIntake())
    , m_led(subsystemAccess.GetLED())
    , m_drive(subsystemAccess.GetDrive())
    , m_vision(subsystemAccess.GetVision())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetShooter(), &subsystemAccess.GetIntake(), &subsystemAccess.GetLED(), &subsystemAccess.GetLED(), &subsystemAccess.GetVision()});

	  wpi::log::DataLog& log = subsystemAccess.GetLogger();
    m_logStartCommand = wpi::log::BooleanLogEntry(log, "/StopAllCommand/startCommand");
}

void StopAllCommand::Initialize()
{
  m_logStartCommand.Append(true);
  m_shooterSubsystem.Stop();
  m_intakeSubsystem.Stop();
  m_drive.Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false);
  m_led.SetAnimation(m_led.GetDefaultColor(), LEDSubsystem::kSolid);
  m_led.SetRobotBusy(false);
  m_vision.DisableShooterLEDs();
}

void StopAllCommand::Execute()
{
}

bool StopAllCommand::IsFinished()
{
  return true;
}

void StopAllCommand::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}
