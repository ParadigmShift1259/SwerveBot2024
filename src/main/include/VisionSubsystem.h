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

class VisionSubsystem : public frc2::SubsystemBase
{
    public:
    VisionSubsystem(/* args */);
    void Periodic();

    std::vector<double> m_net_buffer{2};

    private:

    std::vector<double> m_zero_vector = {42.0, 42.0, 42.0, 92, 10, 22};

    std::shared_ptr<nt::NetworkTable> m_net_table = 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-twoplus");

  wpi::log::DoubleLogEntry m_logRobotPoseX;
  wpi::log::DoubleLogEntry m_logRobotPoseY;
  wpi::log::DoubleLogEntry m_logRobotPoseTheta;
};