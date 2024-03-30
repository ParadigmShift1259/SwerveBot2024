#include "commands/GoToAzimuthCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

const double c_tolerance = 0.1;



GoToAzimuthCommand::GoToAzimuthCommand(ISubsystemAccess& subsystemAccess)
    : m_driveSubsystem(subsystemAccess.GetDrive())
    , m_visionSubsystem(subsystemAccess.GetVision())
    , m_led(subsystemAccess.GetLED())
{
    AddRequirements(frc2::Requirements{&subsystemAccess.GetDrive(), &subsystemAccess.GetVision(), &subsystemAccess.GetLED()});
}

void GoToAzimuthCommand::Initialize()
{
    m_led.SetCurrentAction(LEDSubsystem::CurrentAction::kShootMovement);
    m_led.SetAnimation(c_colorWhite, LEDSubsystem::Animation::kFlow);
    m_visionSubsystem.SetShooterAnglePipeline();
    m_yawError = m_visionSubsystem.GetYawError();
    m_commandedAzimuth = units::radian_t{m_yawError} + m_driveSubsystem.GetGyroAzimuth();
}

void GoToAzimuthCommand::Execute()
{
    m_yawError = m_visionSubsystem.GetYawError();
    auto yawRadians = (m_yawError / 180.0) * std::numbers::pi;
    yawRadians = std::clamp(yawRadians, -std::numbers::pi / 2.0, std::numbers::pi / 2.0);
    m_rotInput = units::radians_per_second_t{4.0 * sin(yawRadians)};
    // printf("rotinput %.3f\n", m_rotInput.value());
    m_driveSubsystem.Drive(0.0_mps, 0.0_mps, m_rotInput, false);
}

bool GoToAzimuthCommand::IsFinished()
{   
    auto rot = m_driveSubsystem.GetPose().Rotation().Radians();
    // printf("rotation %.3f commandedpose %.3f\n", rot.value(), m_commandedAzimuth.value());
    bool finished = (fabs(m_rotInput.value()) < 0.17) || (fabs(m_yawError) < 1.0) || fabs(rot.value() - m_commandedAzimuth.value()) < c_tolerance;

    // if (finished) 
    // {
    //     printf("tv %s commanded %.3f rot %.3f \n"
    //     , m_visionSubsystem.IsValidShooter() ? "true" : "false"
    //     , m_commandedAzimuth
    //     , rot
    //     );
    // }

    // return abs(c_targetY - y) < c_tolerance && abs(c_targetX - x) < c_tolerance;
    return finished;
}

void GoToAzimuthCommand::End(bool interrupted)
{
    m_led.SetAnimation(c_colorWhite, LEDSubsystem::kStrobe);
    frc::SmartDashboard::PutBoolean("IsAiming", false);
    // m_visionSubsystem.SetShooterPositionPipeline();
    m_driveSubsystem.RotationDrive(0.0_mps, 0.0_mps, 0.0_rad, false);
}
