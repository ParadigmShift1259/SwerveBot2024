#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem()
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_logRobotAlliPoseX = wpi::log::DoubleLogEntry(log, "/vision/robotAlliPoseX");
  m_logRobotAlliPoseY = wpi::log::DoubleLogEntry(log, "/vision/robotAlliPoseY");
  m_logRobotAlliPoseTheta = wpi::log::DoubleLogEntry(log, "/vision/robotAlliPoseTheta");
  m_logLL_Latency = wpi::log::DoubleLogEntry(log, "/vision/LL_Latency");
  m_logRobotPoseX = wpi::log::DoubleLogEntry(log, "/vision/robotPoseX");
  m_logRobotPoseY = wpi::log::DoubleLogEntry(log, "/vision/robotPoseY");
  m_logRobotPoseTheta = wpi::log::DoubleLogEntry(log, "/vision/robotPoseTheta");
  m_logtx = wpi::log::DoubleLogEntry(log, "/vision/tx");
  m_logty = wpi::log::DoubleLogEntry(log, "/vision/ty");
  m_logta = wpi::log::DoubleLogEntry(log, "/vision/ta");
  m_logts = wpi::log::DoubleLogEntry(log, "/vision/ts");
  
  frc::SmartDashboard::PutNumber("ATTSAngle", 6.53);

  frc::SmartDashboard::PutNumber("VisionShotAngle", m_shotAngle);
}

void VisionSubsystem::Periodic()
{
  m_isValid = m_net_table->GetNumber("tv", 0) == 1.0;
  //printf(" VisionSubsystem::Periodic() valid %d tv %.3f\n", m_isValid, m_net_table->GetNumber("tv", 0));
  if (m_isValid)
  {
      m_net_buffer = m_net_table->GetNumberArray(m_bIsBlue ? "botpose_wpiblue" : "botpose_wpired", m_zero_vector);
      m_logRobotAlliPoseX.Append(m_net_buffer[eX]);
      m_logRobotAlliPoseY.Append(m_net_buffer[eY]);
      m_logRobotAlliPoseTheta.Append(m_net_buffer[eYaw]);
      m_logLL_Latency.Append(m_net_buffer[eLatency]);

      m_net_buffer = m_net_table->GetNumberArray("botpose", m_zero_vector);
      m_logRobotPoseX.Append(m_net_buffer[eX]);
      m_logRobotPoseY.Append(m_net_buffer[eY]);
      m_logRobotPoseTheta.Append(m_net_buffer[eYaw]);

      m_ty = m_net_table->GetNumber("ty", 0.0);
      m_tx = m_net_table->GetNumber("tx", 0.0);
      m_logtx.Append(m_tx);
      m_logty.Append(m_ty);
      m_logta.Append(m_net_table->GetNumber("ta", 0.0));
      m_logts.Append(m_net_table->GetNumber("ts", 0.0));

      constexpr double c_limelightMountAngle = 42.0;
      //double aprilTagToSpeakerAngle = frc::SmartDashboard::GetNumber("ATTSAngle", 6.53);
      double yOffset = frc::SmartDashboard::GetNumber("ATTSAngle", 10.2);

      double aprilTagToSpeakerAngle = -0.291 * m_ty + yOffset;//10.2;
      m_shotAngle = (c_limelightMountAngle + m_ty) + aprilTagToSpeakerAngle;
      frc::SmartDashboard::PutNumber("VisionShotAngle", m_shotAngle);
      // printf("m_tx %.3f\n", m_tx);
      double yawError = -1.0 * (m_tx / 180.0) * std::numbers::pi;
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
    double yawError = 0.0;
    frc::SmartDashboard::PutNumber("SteerAdjustment", yawError);
  }
}

units::degree_t VisionSubsystem::GetShotAngle()
{
  return units::degree_t{m_shotAngle};
}
