// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <cameraserver/CameraServer.h>

wpi::log::DoubleLogEntry logMatchTime;

void Robot::RobotInit()
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();
  logMatchTime = wpi::log::DoubleLogEntry(log, "/robot/matchTime");
  frc::CameraServer::StartAutomaticCapture();
}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
  m_container.Periodic();
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic()
{
}

void Robot::DisabledExit()
{
}

void Robot::AutonomousInit()
{
  frc::DataLogManager::LogNetworkTables(false);
  frc::DataLogManager::Start();
  // Record both DS control and joystick data
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  // m_container.SetIsAutoRunning(true);
  m_autonomousCommand = m_container.GetAutonomousCommand();

  // if (m_autonomousCommand) {
  //   m_autonomousCommand->Schedule();
  // }
}

void Robot::AutonomousPeriodic()
{
}

void Robot::AutonomousExit()
{
  m_container.SetIsAutoRunning(false);
}

void Robot::TeleopInit()
{
  m_container.SetIsAutoRunning(false);
  if (m_hasAutoRun == false)
  {
    frc::DataLogManager::Start();
  }

  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic()
{

}

void Robot::TeleopExit()
{

}

void Robot::TestInit() 
{
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic()
{

}

void Robot::TestExit()
{

}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
