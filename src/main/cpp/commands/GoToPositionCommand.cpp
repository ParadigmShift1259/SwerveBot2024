#include "commands/GoToPositionCommand.h"

const double c_targetPodiumX = (2.896_m - 14.75_in).value();
const double c_targetPodiumY = 4.106;
const double c_tolerance = 0.1;

const double c_targetSpeakerX = 1.26;
const double c_targetSpeakerY = 5.35;


GoToPositionCommand::GoToPositionCommand(ISubsystemAccess& subsystemAccess, bool bGoToPodium)
    : m_driveSubsystem(subsystemAccess.GetDrive())
    , m_visionSubsystem(subsystemAccess.GetVision())
    , m_targetX(bGoToPodium ? c_targetPodiumX : c_targetSpeakerX)
    , m_targetY(bGoToPodium ? c_targetPodiumY : c_targetSpeakerY)
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetDrive(), &subsystemAccess.GetVision()});
}

void GoToPositionCommand::Initialize()
{

}

void GoToPositionCommand::Execute()
{
    const double c_maxX = 3.0;
    const double c_maxY = 3.0;
    auto x = m_visionSubsystem.GetX();
    auto y = m_visionSubsystem.GetY();
    auto xInput = 0.0;
    auto yInput = 0.0;
    auto xDiff = fabs(m_targetX - x);
    auto yDiff = fabs(m_targetY - y);
    auto xSpeed = 0.0_mps;
    auto ySpeed =0.0_mps;
    
    if (m_visionSubsystem.IsValid())
    {
        if (xDiff >= c_tolerance && xDiff < c_maxX)
        {
            xInput = (m_targetX - x) / c_maxX;
        }

        if (yDiff >= c_tolerance && yDiff < c_maxY)
        {
            yInput = (m_targetY - y) / c_maxY;
        }

        xSpeed = xInput * m_driveSubsystem.m_currentMaxSpeed; 
        ySpeed = yInput * m_driveSubsystem.m_currentMaxSpeed;         

        
        m_driveSubsystem.Drive(xSpeed, ySpeed, 0.0_rad_per_s, false);
    }

    printf("tv %s x %.3f y %.3f xDiff %.3f yDiff %.3f xInput %.3f yInput %.3f xSpeed %.3f yspeed %.3f \n"
        , m_visionSubsystem.IsValid() ? "true" : "false"
        , x
        , y
        , xDiff
        , yDiff
        , xInput
        , yInput
        , xSpeed.value()
        , ySpeed.value());

}

bool GoToPositionCommand::IsFinished()
{
    auto x = m_visionSubsystem.GetX();
    auto y = m_visionSubsystem.GetY();
    auto xDiff = fabs(m_targetX - x);
    auto yDiff = fabs(m_targetY - y);

    bool finished = fabs(m_targetY - y) < c_tolerance && fabs(m_targetX - x) < c_tolerance;

    if (finished) 
    {
        printf("tv %s x %.3f y %.3f xDiff %.3f yDiff %.3f \n"
        , m_visionSubsystem.IsValid() ? "true" : "false"
        , x
        , y
        , xDiff
        , yDiff
        );
    }

    // return abs(c_targetY - y) < c_tolerance && abs(c_targetX - x) < c_tolerance;
    return finished;
}

void GoToPositionCommand::End(bool interrupted)
{
    m_driveSubsystem.Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false);
    printf("ending \n");
}
