#include "GoToPodiumCommand.h"

const double c_targetX = 2.896;
const double c_targetY = 4.106;
const double c_tolerance = 0.1;

GoToPodiumCommand::GoToPodiumCommand(ISubsystemAccess& subsystemAccess)
    : m_driveSubsystem(subsystemAccess.GetDrive())
    , m_visionSubsystem(subsystemAccess.GetVision())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetDrive(), &subsystemAccess.GetVision()});
}

void GoToPodiumCommand::Initialize()
{

}

void GoToPodiumCommand::Execute()
{
    const double c_maxX = 3.0;
    const double c_maxY = 3.0;

    if (m_visionSubsystem.IsValid())
    {
        auto x = m_visionSubsystem.GetX();
        auto y = m_visionSubsystem.GetY();
        auto xInput = 0.0;
        auto yInput = 0.0;

        if (abs(x - c_targetX) >= c_tolerance && x > -c_maxX && x < c_maxX)
        {
            xInput = (x - c_targetX) / c_maxX;
        }

        if (abs(y - c_targetY) >= c_tolerance && y > -c_maxY && y < c_maxY)
        {
            yInput = (y - c_targetY) / c_maxY;
        }

        const auto xSpeed = xInput * m_driveSubsystem.m_currentMaxSpeed; 
        auto ySpeed = yInput * m_driveSubsystem.m_currentMaxSpeed;         

        printf("x %.3f y %.3f xSpeed %.3f yspeed %.3f \n", xInput, yInput, xSpeed.value(), ySpeed.value());

        // m_driveSubsystem.Drive(xSpeed, ySpeed, 0.0_rad_per_s, false);
    }
}

bool GoToPodiumCommand::IsFinished()
{
    auto x = m_visionSubsystem.GetX();
    auto y = m_visionSubsystem.GetY();

    return abs(y - c_targetY) < c_tolerance && abs(x - c_targetX) < c_tolerance;
}

void GoToPodiumCommand::End(bool interrupted)
{
    m_driveSubsystem.Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false);
}
