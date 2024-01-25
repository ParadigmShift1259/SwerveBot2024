// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/WaitCommand.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
// #include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <unordered_map>

#include "ISubsystemAccess.h"
#include "DriveSubsystem.h"

using namespace frc;
using namespace frc2;
// using namespace pathplanner;

class RobotContainer : public ISubsystemAccess
{
public:
  RobotContainer();
  
  CommandPtr GetAutonomousCommand();
  enum EAutoPath
  {
      kAutoPathPlaceAndBalance
    , kAutoPathPlaceAndExitTags1Or8
    , kAutoPathPlaceAndExitTags3Or6
    //, kAutoPath
    // Keep the emun in sync with the LUT
  };
  std::vector<std::string> m_pathPlannerLUT
  { 
      "PlaceAndBalance"       // These strings are the names of the PathPlanner .path files
    , "PlaceAndExitTags1Or8" 
    , "PlaceAndExitTags3Or6"
  };
  frc::SendableChooser<EAutoPath> m_chooser;
  void SetIsAutoRunning(bool isAutoRunning) { m_isAutoRunning = isAutoRunning; }

  void Periodic();

  // ISubsystemAcces Implementation
  IDriveSubsystem&        GetDrive() override { return m_drive; }
  // VisionSubsystem&        GetVision() override { return m_vision; }

  wpi::log::DataLog&         GetLogger() override { return DataLogManager::GetLog(); }

private:
  void SetDefaultCommands();
  void ConfigureBindings();
  void ConfigPrimaryButtonBindings();
  void ConfigSecondaryButtonBindings();
  void ConfigSecondaryButtonBindingsNewWay();
  SwerveControllerCommand<4>* GetSwerveCommandPath(Trajectory trajectory); 
  void PrintTrajectory(Trajectory& trajectory);

 private:
  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_drive;
  // VisionSubsystem m_vision;

  CommandXboxController m_primaryController{0};
  CommandXboxController m_secondaryController{1};
  // SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  // SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s, -3 / 2_s};
  SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s, -3 / 3_s};
  SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  // TODO Make sure field relative starts how the drive team wants
  bool m_fieldRelative = false; //true;
  
  InstantCommand m_toggleFieldRelative{[this] { m_fieldRelative = !m_fieldRelative; }, {}};
  InstantCommand m_toggleSlowSpeed{[this] { GetDrive().ToggleSlowSpeed(); }, {&m_drive}};
  // frc2::InstantCommand m_runCompressor{[this] { m_compressor.EnableDigital(); m_bRunningCompressor = true;}, {} };

#ifdef USE_TEST_BUTTONS
  InstantCommand m_toggleDriveStraight{[this] 
  { 
    m_DriveStraightHook = !m_DriveStraightHook;
    printf("m_DriveStraightHook %s\n", m_DriveStraightHook ? "true" : "false");
  }, {} };
#endif

  InstantCommand m_wheelsForward{[this] { GetDrive().WheelsForward(); }, {&m_drive} };
  InstantCommand m_wheelsLeft{[this] { GetDrive().WheelsLeft(); }, {&m_drive} };
  InstantCommand m_wheelsBackward{[this] { GetDrive().WheelsBackward(); }, {&m_drive} };
  InstantCommand m_wheelsRight{[this] { GetDrive().WheelsRight(); }, {&m_drive} };

  InstantCommand m_OverrideOn{[this] { GetDrive().SetOverrideXboxInput(true); }, {&m_drive} };
  InstantCommand m_OverrideOff{[this] { GetDrive().SetOverrideXboxInput(false); }, {&m_drive} };
  // std::unordered_map<std::string, std::shared_ptr<frc2::Command>> m_eventMap;
  // SwerveAutoBuilder m_autoBuilder;

  bool m_isAutoRunning = false;
  bool m_DriveStraightHook = false;
};
