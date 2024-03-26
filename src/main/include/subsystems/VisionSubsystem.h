#pragma once

#include <vector>
#include <wpi/DataLog.h>

#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/filter/LinearFilter.h>
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
    VisionSubsystem();
    
    void Periodic() override;
    bool IsValidShooter() { return m_isValidShooter; }
    bool IsValidAmp() { return m_isValidAmp; }
    double GetX() { return m_netBufferAlli[eX]; }
    double GetY() { return m_netBufferAlli[eY]; }
    double GetYaw() { return m_netBufferAlli[eYaw]; }
    double GetLatency() { return m_netBufferAlli[eLatency]; }
    int GetTagId() { return m_tidAmp; }
    units::degree_t GetShotAngle();

  private:
    void PeriodicShooter();
    void PeriodicAmp();

    bool m_isValidShooter = false;
    bool m_isValidAmp = false;
    std::vector<double> m_netBufferField{2};
    std::vector<double> m_netBufferAlli{2};
    std::vector<double> m_zero_vector = {42.0, 42.0, 42.0, 92, 10, 22};

    std::shared_ptr<nt::NetworkTable> m_netTableShooter = 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-shooter");

    std::shared_ptr<nt::NetworkTable> m_netTableAmp = 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-amp");
  
  bool m_bIsBlue = false;
  double m_tyShooter = 0.0;
  double m_txShooter = 0.0;
  double m_tyAmp = 0.0;
  double m_txAmp = 0.0;
  int m_tidAmp = 0;
  double m_shotAngle = 0.0;

  double c_defaultAimP = -0.1;
  double c_minAimCommanded = 0.05;

  frc::LinearFilter<double> m_elevationAngleFilter = frc::LinearFilter<double>::MovingAverage(25);

  wpi::log::DoubleLogEntry m_logRobotAlliPoseX;
  wpi::log::DoubleLogEntry m_logRobotAlliPoseY;
  wpi::log::DoubleLogEntry m_logRobotAlliPoseTheta;
  wpi::log::DoubleLogEntry m_logRobotPoseX;
  wpi::log::DoubleLogEntry m_logRobotPoseY;
  wpi::log::DoubleLogEntry m_logRobotPoseTheta;
  wpi::log::DoubleLogEntry m_logLL_Latency;
  wpi::log::DoubleLogEntry m_logtxShooter;
  wpi::log::DoubleLogEntry m_logtyShooter;
  wpi::log::DoubleLogEntry m_logtxAmp;
  wpi::log::DoubleLogEntry m_logtyAmp;
  wpi::log::IntegerLogEntry m_logtidAmp;
};