#include "commands/ClimbCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

const double c_targetPodiumX = (2.896_m - 14.75_in).value();
const double c_targetPodiumY = 4.106;
const double c_tolerance = 0.1;

const double c_targetSpeakerX = 1.26;
const double c_targetSpeakerY = 5.35;


ClimbCommand::ClimbCommand(ISubsystemAccess& subsystemAccess, ClimberSubsystem::Position pos)
    : m_climber(subsystemAccess.GetClimber())
    , m_led(subsystemAccess.GetLED())
    , m_shooter(subsystemAccess.GetShooter())
    , m_intake(subsystemAccess.GetIntake())
{
    m_position = pos;
    // frc::SmartDashboard::PutNumber("Plopper Position", 0.0);
    AddRequirements(frc2::Requirements
    {
          &subsystemAccess.GetClimber()
        , &subsystemAccess.GetLED()
        , &subsystemAccess.GetShooter()
        , &subsystemAccess.GetIntake()
    });
}

void ClimbCommand::Initialize()
{
    m_timer.Reset();
    m_timer.Start();
    m_shooter.GoToElevation(33.0_deg);
    m_intake.GoToPosition(25.5);
    m_shooter.DisableSyncToGyro();
    m_led.SetRobotBusy(true);
    if (m_position == ClimberSubsystem::kHighPosition)
    {
        m_led.SetAnimation(c_colorPink, LEDSubsystem::kFlow);
        m_positionTurns = frc::SmartDashboard::GetNumber("ClimbHiTurns", 170.0);
    } 
    else 
    {
        m_led.SetAnimation(c_colorPink, LEDSubsystem::kStrobe);
        m_positionTurns = frc::SmartDashboard::GetNumber("ClimbParkTurns", 0.0);
    }
}

void ClimbCommand::Execute()
{
    m_climber.GoToPosition(m_positionTurns);
}

bool ClimbCommand::IsFinished()
{
    if (m_position == ClimberSubsystem::kHighPosition){
        return true;
    }
    else 
    {
        if (m_timer.HasElapsed(1.5_s)) {
            return true;
        }
    }
    return false;
}

void ClimbCommand::End(bool interrupted)
{
    m_led.SetAnimation(m_led.GetDefaultColor(), LEDSubsystem::kSolid);
    m_led.SetRobotBusy(false);
    // m_climber.Stop();
}