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
#include <unordered_map>

#include "ISubsystemAccess.h"

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
    , kAutoPathOneNoteLeft
    , kAutoPathOneNoteRight
    , kAutoPathCollectNotes
    , kAutoPathDoNothing
    // Keep the emun in sync with the LUT
  };
  std::vector<std::string> m_pathPlannerLUT
  { 
      "4NoteAuto" 
    , "3NoteAuto"
    , "2NoteAuto" 
    , "1NoteLeftBlueAuto"
    , "1NoteRightBlueAuto"
    , "CollectNotesAuto"
    , "DoNothingAuto"       // These strings are the names of the PathPlanner .path files
  };
  frc::SendableChooser<EAutoPath> m_chooser;
  void SetIsAutoRunning(bool isAutoRunning) { m_isAutoRunning = isAutoRunning; }

  void Periodic();
  void ConfigureRobotLEDs();

  // ISubsystemAcces Implementation
  DriveSubsystem&        GetDrive() override { return m_drive; }
  VisionSubsystem&       GetVision() override { return m_vision; }

  IntakeSubsystem&        GetIntake() override { return m_intake; }
  ShooterSubsystem&        GetShooter() override { return m_shooter; }
  ClimberSubsystem&        GetClimber() override { return m_climber; }
  LEDSubsystem&        GetLED() override { return m_led; }

  wpi::log::DataLog&         GetLogger() override { return DataLogManager::GetLog(); }

  LEDSubsystem::Color c_colorPink = LEDSubsystem::CreateColor(80, 10, 15 , 0);//(255, 141, 187, 0);
  LEDSubsystem::Color c_colorGreen = LEDSubsystem::CreateColor(13, 80, 0, 0);//(133, 240, 45, 0);
  LEDSubsystem::Color c_colorBlack = LEDSubsystem::CreateColor(0, 0, 0, 0);
  LEDSubsystem::Color c_colorOrange = LEDSubsystem::CreateColor(43, 6, 0, 255);
  LEDSubsystem::Color c_colorWhite = LEDSubsystem::CreateColor(255, 255, 255, 10);

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
  LEDSubsystem m_led;
  
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
  SlewRateLimiter<units::scalar> m_yawRotationLimiter{3 / 1_s};

  // TODO Make sure field relative starts how the drive team wants
  bool m_fieldRelative = false; //true;
  
  InstantCommand m_toggleFieldRelative{[this] { m_fieldRelative = !m_fieldRelative; }, {}};
  InstantCommand m_toggleSlowSpeed{[this] { GetDrive().ToggleSlowSpeed(); }, {&m_drive}};
  InstantCommand m_SetUseCloseShot{[this] { m_shooter.SetUseLongShot(false); }, {&m_shooter}};
  InstantCommand m_SetUseLongShot{[this] { m_shooter.SetUseLongShot(true); }, {&m_shooter}};
  
  // frc2::InstantCommand m_runCompressor{[this] { m_compressor.EnableDigital(); m_bRunningCompressor = true;}, {} };
  InstantCommand m_resetShooterToStart{[this] { m_shooter.GoToElevation(c_defaultStartPosition); }, {}};
  InstantCommand m_goToElev{[this]
  { 
    units::degree_t angle{frc::SmartDashboard::GetNumber("ElevationAngle", 44.0)};
    // units::degree_t angle = m_vision.GetShotAngle();
    m_shooter.GoToElevation(angle);
  }, {} };

  InstantCommand m_undershootAngle{[this]
  { 
    units::degree_t angle = m_vision.GetShotAngle();
    if (m_shooter.GetElevation() > angle.value())
    {
      m_shooter.GoToElevation(angle - c_defaultCalibrationOffset);
    }
  }, {} };

 InstantCommand m_jogIntakeIn{[this]
  { 
    m_intake.Set(0.6);
  }, {} };

 InstantCommand m_jogIntakeOut{[this]
  { 
    m_intake.Set(-0.6);
  }, {} };

  InstantCommand m_toggleAmpAllowed{[this]
  { 
    m_vision.ToggleAllowedAmp();
  }, {} };
  
  InstantCommand m_toggleShooterAllowed{[this]
  { 
    m_vision.ToggleAllowedShooter();
  }, {} };

  InstantCommand m_posPipeline{[this]
  { 
    m_vision.SetShooterPositionPipeline();
  }, {} };

  InstantCommand m_visPosFalse{[this]
  { 
    m_vision.SetPositionStarted(false);
  }, {} };

  InstantCommand m_trapRPM{[this]
  { 
    m_shooter.StartOverAndUnder(2300.0);
  }, {} };

  InstantCommand m_angPipeline{[this]
  { 
    m_vision.SetShooterAnglePipeline();
  }, {} };

  InstantCommand m_PrintData{[this]
  { 
    printf("Elev %.3f ShotAngle %.3f FloorDist %.3f\n", m_shooter.GetElevation(), m_vision.GetShotAngle().value(), m_vision.GetFloorDist());
  }, {} };
  InstantCommand m_stopClimb{[this] { m_climber.Stop(); }, {}};

  InstantCommand m_enableGyroSync{[this] { m_shooter.EnableSyncToGyro(); }, {}};

  InstantCommand m_ampPositionIntake{[this]
  { 
    double turns = frc::SmartDashboard::GetNumber("AmpShotTurns", 21);
    m_intake.ExtendIntake(turns);
  }, {} };

  InstantCommand m_ampShootIntake{[this]
  { 
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

#ifdef USE_ORCESTRA
  ctre::phoenix6::Orchestra m_orchestra;	

  InstantCommand m_startOrchestra{[this] { m_orchestra.Play(); }, {} };
  InstantCommand m_endOrchestra{[this] { m_orchestra.Stop(); }, {} };
#endif

  bool m_isAutoRunning = false;
  bool m_DriveStraightHook = false;
};
