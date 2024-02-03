#include "subsystems/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem()
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_logRobotPoseX = wpi::log::DoubleLogEntry(log, "/vision/robotPoseX");
  m_logRobotPoseY = wpi::log::DoubleLogEntry(log, "/vision/robotPoseY");
  m_logRobotPoseTheta = wpi::log::DoubleLogEntry(log, "/vision/robotPoseTheta");
  m_logLL_Latency = wpi::log::DoubleLogEntry(log, "/vision/LL_Latency");
}

void VisionSubsystem::Periodic()
{
  m_isValid = m_net_table->GetNumber("tv", 0);
  if (m_isValid)
  {
      m_net_buffer = m_net_table->GetNumberArray("botpose_wpiblue", m_zero_vector);
      m_logRobotPoseX.Append(m_net_buffer[eX]);
      m_logRobotPoseY.Append(m_net_buffer[eY]);
      m_logRobotPoseTheta.Append(m_net_buffer[eYaw]);
      m_logLL_Latency.Append(m_net_buffer[eLatency]);
  }
}