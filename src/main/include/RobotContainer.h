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

#include <ctre/phoenix6/Orchestra.hpp>

#include <pathplanner/lib/auto/AutoBuilder.h>
// #include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <unordered_map>

#include "ISubsystemAccess.h"
// #include "subsystems/DriveSubsystem.h"

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
      kAutoPathDefault
    , kAutoPathFourNote = kAutoPathDefault
    , kAutoPathThreeNote
    , kAutoPathTwoNote
    , kAutoPathOneNote
    , kAutoPathExitZone
    , kAutoPathDoNothing
    // Keep the emun in sync with the LUT
  };
  std::vector<std::string> m_pathPlannerLUT
  { 
      "Four Note Auto" 
    , "Three Note Auto"
    , "Two Note Auto" 
    , "One Note Auto"
    , "Exit Zone Auto" 
    , "Do Nothing Auto"       // These strings are the names of the PathPlanner .path files
  };
  frc::SendableChooser<EAutoPath> m_chooser;
  void SetIsAutoRunning(bool isAutoRunning) { m_isAutoRunning = isAutoRunning; }

  void Periodic();

  // ISubsystemAcces Implementation
  DriveSubsystem&        GetDrive() override { return m_drive; }
  VisionSubsystem&       GetVision() override { return m_vision; }

  IntakeSubsystem&        GetIntake() override { return m_intake; }
  ShooterSubsystem&        GetShooter() override { return m_shooter; }
  ClimberSubsystem&        GetClimber() override { return m_climber; }
  AmpSubsystem&        GetAmp() override { return m_amp; }

  wpi::log::DataLog&         GetLogger() override { return DataLogManager::GetLog(); }

private:
  void SetDefaultCommands();
  void ConfigureBindings();
  void ConfigPrimaryButtonBindings();
  void ConfigSecondaryButtonBindings();
  void ConfigButtonBoxBindings();
  SwerveControllerCommand<4>* GetSwerveCommandPath(Trajectory trajectory); 
  void PrintTrajectory(Trajectory& trajectory);

 private:
  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_drive;
  VisionSubsystem m_vision;

  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  ClimberSubsystem m_climber;
  AmpSubsystem m_amp;
  
  double m_shootDelayMs = 1.0;

  CommandXboxController m_primaryController{0};
  CommandXboxController m_secondaryController{1};
#define USE_BUTTON_BOX
#ifdef USE_BUTTON_BOX
  CommandXboxController m_buttonBoxController{2};
#endif
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
  InstantCommand m_resetShooterToStart{[this] { m_shooter.GoToElevation(66_deg); }, {}};
  InstantCommand m_goToElev{[this]
  { 
    units::degree_t angle{frc::SmartDashboard::GetNumber("ElevationAngle", 44.0)};
    m_shooter.GoToElevation(angle);
  }, {} };

  InstantCommand m_ampPositionIntake{[this]
  { 
    double turns = frc::SmartDashboard::GetNumber("AmpShotTurns", 21);
    m_intake.ExtendIntake(turns);
  }, {} };

  InstantCommand m_ampShootIntake{[this]
  { 
    // m_intake.ExtendIntake();
    m_intake.Set(frc::SmartDashboard::GetNumber("AmpShotPercent", -0.6));
  }, {} };

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

  ctre::phoenix6::Orchestra m_orchestra;	

  InstantCommand m_startOrchestra{[this] { m_orchestra.Play(); }, {} };
  InstantCommand m_endOrchestra{[this] { m_orchestra.Stop(); }, {} };

  bool m_isAutoRunning = false;
  bool m_DriveStraightHook = false;
};
