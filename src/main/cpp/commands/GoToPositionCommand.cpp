#include "commands/GoToPositionCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

const units::length::meter_t c_halfRobotSize = 14.75_in;  // Half robot width/length 29.5 / 2 = 14.75

const double c_targetPodiumX = (2.896_m - c_halfRobotSize).value();
const double c_targetPodiumY = 4.106;
const double c_tolerance = 0.02;
const double c_minInput = 0.07;

const double c_targetSpeakerBlueX = 1.26;
const double c_targetSpeakerBlueY = 5.35;

const double c_targetSpeakerRedX = c_targetSpeakerBlueX;
const double c_targetSpeakerRedY = 4.106 - 1.448;

const double c_targetAmpBlueX = (1.933_m - 0.050_m).value();  // 5cm bias on shooter/intake
const double c_targetAmpBlueY = (8.111_m - c_halfRobotSize).value();

const double c_targetAmpRedX = c_targetAmpBlueX;
const double c_targetAmpRedY = c_halfRobotSize.value();

const units::velocity::meters_per_second_t c_defaultGoToAmpMaxSpeed = 4.5_mps;

GoToPositionCommand::GoToPositionCommand(ISubsystemAccess& subsystemAccess, bool bIsBlue)
    : m_driveSubsystem(subsystemAccess.GetDrive())
    , m_visionSubsystem(subsystemAccess.GetVision())
    , m_targetX(bIsBlue ? c_targetAmpBlueX : c_targetAmpRedX)
    , m_targetY(bIsBlue ? c_targetAmpBlueY : c_targetAmpRedY)
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetDrive(), &subsystemAccess.GetVision()});

    frc::SmartDashboard::PutNumber("GoAmpMaxSpd", c_defaultGoToAmpMaxSpeed.value());
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
    
    if (m_visionSubsystem.IsValidAmp())
    {
        if (xDiff >= c_tolerance && xDiff < c_maxX)
        {
            if (m_visionSubsystem.GetTagId() == 5)
            {
                yInput = (m_targetX - x) / c_maxX;
            }
            else
            {
                //xInput = (m_targetX - x) / c_maxX;
                yInput = (x - m_targetX) / c_maxX;
            }
            if (yInput < 0.0)
            {
                yInput = std::min(-c_minInput, yInput);
            }
            else
            {
                yInput = std::max(c_minInput, yInput);
            }
        }

        if (yDiff >= c_tolerance && yDiff < c_maxY)
        {
            //yInput = (m_targetY - y) / c_maxY;
            if (m_visionSubsystem.GetTagId() == 5)
            {
                xInput = (y - m_targetY) / c_maxY;
            }
            else
            {
                xInput = (m_targetY - y) / c_maxY;
            }
            if (xInput < 0.0)
            {
                xInput = std::min(-c_minInput, xInput);
            }
            else
            {
                xInput = std::max(c_minInput, xInput);
            }
        }

        units::velocity::meters_per_second_t maxSpeed = units::velocity::meters_per_second_t{frc::SmartDashboard::GetNumber("GoAmpMaxSpd", c_defaultGoToAmpMaxSpeed.value())};
        xSpeed = xInput * maxSpeed; 
        ySpeed = yInput * maxSpeed;         
        
        m_driveSubsystem.Drive(xSpeed, ySpeed, 0.0_rad_per_s, false);
    }

    printf("tv %s x %.3f y %.3f xDiff %.3f yDiff %.3f xInput %.3f yInput %.3f xSpeed %.3f yspeed %.3f \n"
        , m_visionSubsystem.IsValidAmp() ? "true" : "false"
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
        , m_visionSubsystem.IsValidAmp() ? "true" : "false"
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
}
