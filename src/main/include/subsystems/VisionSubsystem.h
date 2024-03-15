#pragma once

#include <vector>
#include <wpi/DataLog.h>

#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/DataLogManager.h>

#include <units/angle.h>
#include <units/length.h>
#include <iostream>

enum BotPoseIndices
{
    eX
  , eY
  , eZ
  , ePitch
  , eRoll
  , eYaw
  , eLatency
};

class VisionSubsystem : public frc2::SubsystemBase
{
  public:
    VisionSubsystem(/* args */);
    void Periodic() override;
    bool IsValid() { return m_isValid; }
    double GetX() { return m_net_buffer[eX]; }
    double GetY() { return m_net_buffer[eY]; }
    double GetYaw() { return m_net_buffer[eYaw]; }
    double GetLatency() { return m_net_buffer[eLatency]; }
    void SetBlueAlliance(bool bIsBlue) { m_bIsBlue = bIsBlue; } 
    units::degree_t GetShotAngle();

  private:
    bool m_isValid = false;
    std::vector<double> m_net_buffer{2};
    std::vector<double> m_zero_vector = {42.0, 42.0, 42.0, 92, 10, 22};

    std::shared_ptr<nt::NetworkTable> m_net_table = 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-treone");
  
  bool m_bIsBlue = false;
  double m_ty = 0.0;
  double m_shotAngle = 0.0;

  wpi::log::DoubleLogEntry m_logRobotAlliPoseX;
  wpi::log::DoubleLogEntry m_logRobotAlliPoseY;
  wpi::log::DoubleLogEntry m_logRobotAlliPoseTheta;
  wpi::log::DoubleLogEntry m_logRobotPoseX;
  wpi::log::DoubleLogEntry m_logRobotPoseY;
  wpi::log::DoubleLogEntry m_logRobotPoseTheta;
  wpi::log::DoubleLogEntry m_logLL_Latency;
  wpi::log::DoubleLogEntry m_logtx;
  wpi::log::DoubleLogEntry m_logty;
  wpi::log::DoubleLogEntry m_logta;
  wpi::log::DoubleLogEntry m_logts;
};