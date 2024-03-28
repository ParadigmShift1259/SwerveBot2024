#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "subsystems/VisionSubsystem.h"

constexpr double c_limelightShooterMountAngle = 27.0;
constexpr double c_limelightAmpMountAngle = 39.0;
constexpr units::meter_t c_targetHeight = 55.875_in;

VisionSubsystem::VisionSubsystem()
{
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance)
  {
    m_bIsBlue = (alliance.value() == frc::DriverStation::Alliance::kBlue);
  }

  c_distanceToAngleMap.insert(53.25_in,   55.0_deg);
  c_distanceToAngleMap.insert(61.24_in,   50.0_deg);
  c_distanceToAngleMap.insert(76.0_in,    45.0_deg);
  c_distanceToAngleMap.insert(100.0_in,   41.0_deg);
  c_distanceToAngleMap.insert(125.0_in,   37.0_deg);
  c_distanceToAngleMap.insert(150.0_in,   34.0_deg);

  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_logRobotAlliPoseX = wpi::log::DoubleLogEntry(log, "/vision/robotAlliPoseX");
  m_logRobotAlliPoseY = wpi::log::DoubleLogEntry(log, "/vision/robotAlliPoseY");
  m_logRobotAlliPoseTheta = wpi::log::DoubleLogEntry(log, "/vision/robotAlliPoseTheta");
  m_logLL_Latency = wpi::log::DoubleLogEntry(log, "/vision/LL_Latency");
  m_logRobotPoseX = wpi::log::DoubleLogEntry(log, "/vision/robotPoseX");
  m_logRobotPoseY = wpi::log::DoubleLogEntry(log, "/vision/robotPoseY");
  m_logRobotPoseTheta = wpi::log::DoubleLogEntry(log, "/vision/robotPoseTheta");
  m_logtxShooter = wpi::log::DoubleLogEntry(log, "/vision/txShooter");
  m_logtyShooter = wpi::log::DoubleLogEntry(log, "/vision/tyShotter");
  m_logtxAmp = wpi::log::DoubleLogEntry(log, "/vision/txAmp");
  m_logtyAmp = wpi::log::DoubleLogEntry(log, "/vision/tyAmp");
  m_logtidAmp = wpi::log::IntegerLogEntry(log, "/vision/tidAmp");
  
  // frc::SmartDashboard::PutNumber("ATTSAngle", 6.53);
  frc::SmartDashboard::PutNumber("ATTSAngle", 8.2);

  frc::SmartDashboard::PutNumber("VisionShotAngle", m_shotAngle);
}

void VisionSubsystem::Periodic()
{
  PeriodicShooter();
  PeriodicAmp();
}

void VisionSubsystem::PeriodicShooter()
{
  m_isValidShooter = m_netTableShooter->GetNumber("tv", 0) == 1.0;
  if (m_isValidShooter)
  {
      m_netBufferField = m_netTableShooter->GetNumberArray("botpose", m_zero_vector);
      m_logRobotPoseX.Append(m_netBufferField[eX]);
      m_logRobotPoseY.Append(m_netBufferField[eY]);
      m_logRobotPoseTheta.Append(m_netBufferField[eYaw]);

      m_netBufferAlli = m_netTableShooter->GetNumberArray(m_bIsBlue ? "botpose_wpiblue" : "botpose_wpired", m_zero_vector);
      m_logRobotAlliPoseX.Append(m_netBufferAlli[eX]);
      m_logRobotAlliPoseY.Append(m_netBufferAlli[eY]);
      m_logRobotAlliPoseTheta.Append(m_netBufferAlli[eYaw]);
      m_logLL_Latency.Append(m_netBufferAlli[eLatency]);

      m_tyShooter = m_netTableShooter->GetNumber("ty", 0.0);
      m_txShooter = m_netTableShooter->GetNumber("tx", 0.0);
      m_logtxShooter.Append(m_txShooter);
      m_logtyShooter.Append(m_tyShooter);

      auto tyFilteredShooter = m_elevationAngleFilter.Calculate(m_tyShooter);

      //double aprilTagToSpeakerAngle = frc::SmartDashboard::GetNumber("ATTSAngle", 6.53);
      double yOffset = frc::SmartDashboard::GetNumber("ATTSAngle", 10.2);

      double targetAngle = (c_limelightShooterMountAngle + tyFilteredShooter) * std::numbers::pi / 180.0;
      // Linear Fit
      // double aprilTagToSpeakerAngle = -0.291 * tyFilteredShooter + yOffset;//10.2;
      // m_shotAngle = (targetAngle) + aprilTagToSpeakerAngle;
      //floorDistance = height from camera to apriltag / tangent + limelight offset from robot
      m_floorDistance = (45.875 / tan(targetAngle)) + 11.0;
      m_shotAngle = c_distanceToAngleMap[units::inch_t{m_floorDistance}].value();
      frc::SmartDashboard::PutNumber("VisionShotAngle", m_shotAngle);
      m_shotDistance = c_targetHeight.value() / sin(targetAngle);
      frc::SmartDashboard::PutNumber("VisionShotDistance", m_shotDistance);
      // printf("m_tx %.3f\n", m_tx);
      double yawError = -1.0 * (m_txShooter / 180.0) * std::numbers::pi;
      // double steeringAdjust = 0.0f;

      // if (m_tx > 1.0)
      // {
      //   steeringAdjust = c_defaultAimP * yawError - c_minAimCommanded;
      // }
      // else if (m_tx < -1.0)
      // {
      //   steeringAdjust = c_defaultAimP * yawError + c_minAimCommanded;
      // }

      frc::SmartDashboard::PutNumber("SteerAdjustment", yawError);
  }
  else
  {
    m_shotAngle = 0.0;
    m_shotDistance = 0.0;
    double yawError = 0.0;
    frc::SmartDashboard::PutNumber("SteerAdjustment", yawError);
  }
}

void VisionSubsystem::PeriodicAmp()
{
  m_isValidAmp = m_netTableAmp->GetNumber("tv", 0) == 1.0;
  if (m_isValidAmp)
  {
      m_netBufferField = m_netTableAmp->GetNumberArray("botpose", m_zero_vector);
      m_logRobotPoseX.Append(m_netBufferField[eX]);
      m_logRobotPoseY.Append(m_netBufferField[eY]);
      m_logRobotPoseTheta.Append(m_netBufferField[eYaw]);

      m_netBufferAlli = m_netTableAmp->GetNumberArray(m_bIsBlue ? "botpose_wpiblue" : "botpose_wpired", m_zero_vector);
      m_logRobotAlliPoseX.Append(m_netBufferAlli[eX]);
      m_logRobotAlliPoseY.Append(m_netBufferAlli[eY]);
      m_logRobotAlliPoseTheta.Append(m_netBufferAlli[eYaw]);
      m_logLL_Latency.Append(m_netBufferAlli[eLatency]);

      m_tyAmp = m_netTableAmp->GetNumber("ty", 0.0);
      m_txAmp = m_netTableAmp->GetNumber("tx", 0.0);
      m_tidAmp = m_netTableAmp->GetNumber("tid", 0);
      m_logtxAmp.Append(m_txAmp);
      m_logtyAmp.Append(m_tyAmp);
      m_logtidAmp.Append(m_tidAmp);
  }
  // else
  // {
  // }
}

units::degree_t VisionSubsystem::GetShotAngle()
{
  return units::degree_t{m_shotAngle};
}
