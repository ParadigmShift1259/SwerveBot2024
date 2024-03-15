#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem()
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  m_logRobotPoseX = wpi::log::DoubleLogEntry(log, "/vision/robotPoseX");
  m_logRobotPoseY = wpi::log::DoubleLogEntry(log, "/vision/robotPoseY");
  m_logRobotPoseTheta = wpi::log::DoubleLogEntry(log, "/vision/robotPoseTheta");
  m_logLL_Latency = wpi::log::DoubleLogEntry(log, "/vision/LL_Latency");
  m_logtx = wpi::log::DoubleLogEntry(log, "/vision/tx");
  m_logty = wpi::log::DoubleLogEntry(log, "/vision/ty");
  m_logta = wpi::log::DoubleLogEntry(log, "/vision/ta");
  m_logts = wpi::log::DoubleLogEntry(log, "/vision/ts");

  frc::SmartDashboard::PutNumber("VisionShotAngle", m_shotAngle);
}

void VisionSubsystem::Periodic()
{
  m_isValid = m_net_table->GetNumber("tv", 0) == 1.0;
  //printf(" VisionSubsystem::Periodic() valid %d tv %.3f\n", m_isValid, m_net_table->GetNumber("tv", 0));
  if (m_isValid)
  {
      m_net_buffer = m_net_table->GetNumberArray(m_bIsBlue ? "botpose_wpiblue" : "botpose_wpired", m_zero_vector);
      m_logRobotPoseX.Append(m_net_buffer[eX]);
      m_logRobotPoseY.Append(m_net_buffer[eY]);
      m_logRobotPoseTheta.Append(m_net_buffer[eYaw]);
      m_logLL_Latency.Append(m_net_buffer[eLatency]);

      m_ty = m_net_table->GetNumber("ty", 0.0);
      m_logtx.Append(m_net_table->GetNumber("tx", 0.0));
      m_logty.Append(m_ty);
      m_logta.Append(m_net_table->GetNumber("ta", 0.0));
      m_logts.Append(m_net_table->GetNumber("ts", 0.0));

      //m_shotAngle = (45.0 - m_ty) + 13.47;
      m_shotAngle = (45.0 + m_ty) + 6.53;
      frc::SmartDashboard::PutNumber("VisionShotAngle", m_shotAngle);
  }
  else
  {
    m_shotAngle = 0.0;
  }
}

units::degree_t VisionSubsystem::GetShotAngle()
{
  return units::degree_t{m_shotAngle};
}
