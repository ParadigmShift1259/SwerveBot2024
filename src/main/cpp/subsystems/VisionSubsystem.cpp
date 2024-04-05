#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "subsystems/VisionSubsystem.h"

constexpr double c_limelightShooterMountAngle = 27.0;
constexpr double c_limelightAmpMountAngle = 30.0;
constexpr units::meter_t c_targetHeight = 55.875_in;
constexpr double c_limelightShooterPositionPipeline = 0.0;
constexpr double c_limelightShooterAnglePipeline = 1.0;

VisionSubsystem::VisionSubsystem()
{
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance)
  {
    m_bIsBlue = (alliance.value() == frc::DriverStation::Alliance::kBlue);
  }

  // c_distanceToAngleMap.insert(53.25_in,   55.0_deg);
  // c_distanceToAngleMap.insert(61.24_in,   50.0_deg);
  // c_distanceToAngleMap.insert(76.0_in,    45.0_deg);
  // c_distanceToAngleMap.insert(100.0_in,   40.5_deg); // 41.0
  // c_distanceToAngleMap.insert(125.0_in,   36.5_deg); // 37.0
  // c_distanceToAngleMap.insert(150.0_in,   34.0_deg);
  c_distanceToAngleMap.insert( 55.263_in, 55.000_deg);
  c_distanceToAngleMap.insert( 62.329_in, 51.000_deg);
  c_distanceToAngleMap.insert( 74.776_in, 45.250_deg);
  c_distanceToAngleMap.insert( 87.309_in, 41.000_deg);
  c_distanceToAngleMap.insert(101.211_in, 37.000_deg);
  c_distanceToAngleMap.insert(114.604_in, 34.750_deg);
  c_distanceToAngleMap.insert(128.599_in, 32.000_deg);
  c_distanceToAngleMap.insert(142.725_in, 30.750_deg);
  c_distanceToAngleMap.insert(153.514_in, 29.750_deg);
  c_distanceToAngleMap.insert(167.518_in, 28.750_deg);
  c_distanceToAngleMap.insert(182.300_in, 28.250_deg);
  c_distanceToAngleMap.insert(199.069_in, 26.750_deg);

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

  frc::SmartDashboard::PutNumber("VisionShotAngle", m_shotAngle);
  frc::SmartDashboard::PutNumber("VisionShotOffset", m_visionShotOffset);

  frc::SmartDashboard::PutBoolean("AllowedShooter", m_isAllowedShooter);
  frc::SmartDashboard::PutBoolean("AllowedAmp", m_isAllowedAmp);
}

void VisionSubsystem::Periodic()
{
  PeriodicShooter();
  PeriodicAmp();
  m_isAllowedShooter = frc::SmartDashboard::GetBoolean("AllowedShooter", m_isAllowedShooter);
  m_isAllowedAmp = frc::SmartDashboard::GetBoolean("AllowedAmp", m_isAllowedAmp);
  m_visionShotOffset = frc::SmartDashboard::GetNumber("VisionShotOffset", 2.71);
}

void VisionSubsystem::PeriodicShooter()
{
  m_isValidShooter = m_netTableShooter->GetNumber("tv", 0) == 1.0;
  if (m_isValidShooter && m_isAllowedShooter)
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

      double targetAngle = (c_limelightShooterMountAngle + tyFilteredShooter) * std::numbers::pi / 180.0;
      
      // floorDistance = height from camera to apriltag / tangent + limelight offset from robot
      m_floorDistance = (45.875 / tan(targetAngle)) + 11.0;
      frc::SmartDashboard::PutNumber("VisionFloorDist", m_floorDistance);
      m_shotAngle = c_distanceToAngleMap[units::inch_t{m_floorDistance}].value() + m_visionShotOffset;
      frc::SmartDashboard::PutNumber("VisionShotAngle", m_shotAngle);
      m_shotDistance = c_targetHeight.value() / sin(targetAngle);
      frc::SmartDashboard::PutNumber("VisionShotDistance", m_shotDistance);
      m_yawError = m_txShooter;
      frc::SmartDashboard::PutNumber("VisionYawError", m_yawError);
  }
  else
  {
    m_shotAngle = 0.0;
    m_shotDistance = 0.0;
    m_yawError = 0.0;
  }
}

void VisionSubsystem::PeriodicAmp()
{
  m_isValidAmp = m_netTableAmp->GetNumber("tv", 0) == 1.0;
  if (m_isValidAmp && m_isAllowedAmp)
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

void VisionSubsystem::SetShooterPositionPipeline() 
{
  m_netTableShooter->PutNumber("pipeline", c_limelightShooterPositionPipeline);
}

void VisionSubsystem::SetShooterAnglePipeline() 
{
  m_netTableShooter->PutNumber("pipeline", c_limelightShooterAnglePipeline);
}