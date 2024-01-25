#include "VisionSubsystem.h"

VisionSubsystem::VisionSubsystem()
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_logRobotPoseX = wpi::log::DoubleLogEntry(log, "/vision/robotPoseX");
  m_logRobotPoseY = wpi::log::DoubleLogEntry(log, "/vision/robotPoseY");
  m_logRobotPoseTheta = wpi::log::DoubleLogEntry(log, "/vision/robotPoseTheta");
}

void VisionSubsystem::Periodic()
{
  if (m_net_table->GetNumber("tv", 0))
  {
      m_net_buffer = m_net_table->GetNumberArray("botpose_wpiblue", m_zero_vector);
      m_logRobotPoseX.Append(m_net_buffer[0]);
      m_logRobotPoseY.Append(m_net_buffer[1]);
      m_logRobotPoseTheta.Append(m_net_buffer[5]);
  }
}