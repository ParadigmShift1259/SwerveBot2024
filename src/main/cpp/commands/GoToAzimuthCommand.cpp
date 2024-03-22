#include "commands/GoToAzimuthCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

const double c_targetPodiumX = (2.896_m - 14.75_in).value();
const double c_targetPodiumY = 4.106;
const double c_tolerance = 0.1;

const double c_targetSpeakerX = 1.26;
const double c_targetSpeakerY = 5.35;


GoToAzimuthCommand::GoToAzimuthCommand(ISubsystemAccess& subsystemAccess)
    : m_driveSubsystem(subsystemAccess.GetDrive())
    , m_visionSubsystem(subsystemAccess.GetVision())
    // , m_targetX(bGoToPodium ? c_targetPodiumX : c_targetSpeakerX)
    // , m_targetY(bGoToPodium ? c_targetPodiumY : c_targetSpeakerY)
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetDrive(), &subsystemAccess.GetVision()});
}

void GoToAzimuthCommand::Initialize()
{
    double adjustment = frc::SmartDashboard::GetNumber("SteerAdjustment", 0.0);
    // double rotInput = sin(adjustment);
    m_rot = units::radians_per_second_t{frc::SmartDashboard::GetNumber("AdjustRotation", 0.0)};//m_rotLimiter.Calculate(rotInput) * kMaxAngularSpeed;
    frc::SmartDashboard::PutBoolean("IsAiming", true);
    units::angle::radian_t poseRadians = m_driveSubsystem.GetGyroAzimuth();//GetPose().Rotation().Radians();
    // double multiplier = frc::SmartDashboard::GetNumber("yawsign", 1.0);
    frc::SmartDashboard::PutNumber("startposeradians", poseRadians.value());
    m_commandedAzimuth = units::angle::radian_t{poseRadians.value() + adjustment};
    frc::SmartDashboard::PutNumber("commandedposition", m_commandedAzimuth.value());
    // printf("start angle %.3f adjustment %.3f commanded angle %.3f", poseRadians.value(), adjustment, m_commandedAzimuth.value());
}

void GoToAzimuthCommand::Execute()
{
    m_rot = units::radians_per_second_t{frc::SmartDashboard::GetNumber("AdjustRotation", 0.0)};
    m_driveSubsystem.Drive(0.0_mps, 0.0_mps, m_rot, false);
    // auto rot = m_driveSubsystem.GetPose().Rotation().Radians();
    // printf("rotation %.3f commandedpose %.3f\n", rot.value(), m_commandedAzimuth.value());
}

bool GoToAzimuthCommand::IsFinished()
{
    // return false;
    
    auto rot = m_driveSubsystem.GetPose().Rotation().Radians();
    // printf("rotation %.3f commandedpose %.3f\n", rot.value(), m_commandedAzimuth.value());
    bool finished = fabs(rot.value() - m_commandedAzimuth.value()) < c_tolerance;

    if (finished) 
    {
        printf("tv %s commanded %.3f rot %.3f \n"
        , m_visionSubsystem.IsValid() ? "true" : "false"
        , m_commandedAzimuth
        , rot
        );
    }

    // return abs(c_targetY - y) < c_tolerance && abs(c_targetX - x) < c_tolerance;
    return finished;
}

void GoToAzimuthCommand::End(bool interrupted)
{
    frc::SmartDashboard::PutBoolean("IsAiming", false);
    m_driveSubsystem.RotationDrive(0.0_mps, 0.0_mps, 0.0_rad, false);
}
