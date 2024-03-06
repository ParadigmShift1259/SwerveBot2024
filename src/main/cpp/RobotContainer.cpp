// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/GoToPositionCommand.h"
#include "commands/IntakeStop.h"
#include "commands/IntakeRelease.h"
#include "commands/IntakeAdjust.h"
#include "commands/IntakeIngest.h"
#include "commands/IntakeDeploy.h"
#include "commands/StopAllCommand.h"
#include "commands/PreShootCommand.h"
#include "commands/ShootCommand.h"
#include "commands/PlopperGoToPositionCommand.h"
#include "commands/PlopperShootCommand.h"
#include "commands/AmpIntakeCommand.h"
#include "commands/GoToElevationCommand.h"
#include "commands/IntakeGoToPositionCommand.h"
#include "commands/IntakeTransfer.h"
#include "commands/AmpShootCommand.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/DriverStation.h>

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <pathplanner/lib/path/PathPlannerTrajectory.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPoint.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <pathplanner/lib/auto/NamedCommands.h>

using namespace pathplanner;

RobotContainer::RobotContainer() 
  : m_drive()
  , m_orchestra("output.chrp")
{
  // NamedCommands::registerCommand("PreShootClose", std::move(PreShootCommand(*this, 1_m).ToPtr()));
  // NamedCommands::registerCommand("ShootClose", std::move(ShootCommand(*this).ToPtr()));

  NamedCommands::registerCommand("ShootClose", std::move(
    frc2::SequentialCommandGroup{
        PreShootCommand(*this, 1_m)
      , frc2::WaitCommand(0.25_s)
      , ShootCommand(*this)
    }.ToPtr()));

  NamedCommands::registerCommand("ShootStop", std::move(StopAllCommand(*this).ToPtr()));

  NamedCommands::registerCommand("ShootFar", std::move(
    frc2::SequentialCommandGroup{
        frc2::WaitCommand(0.25_s)
      ,  PreShootCommand(*this, 5_m)
      , frc2::WaitCommand(1.0_s)
      , ShootCommand(*this)
    }.ToPtr()));

  NamedCommands::registerCommand("Intake Note", std::move(
    frc2::SequentialCommandGroup{
        IntakeIngest(*this)
      , frc2::WaitCommand(0.25_s)
      , IntakeTransfer(*this)
    }.ToPtr()));

  SetDefaultCommands();
  ConfigureBindings();

  m_chooser.SetDefaultOption(m_pathPlannerLUT[kAutoPathDefault], EAutoPath::kAutoPathDefault);
  for (int i = 1; i < (int)m_pathPlannerLUT.size(); i++)
  {
    m_chooser.AddOption(m_pathPlannerLUT[i], (EAutoPath)i);
  }
  frc::SmartDashboard::PutData("Auto Path", &m_chooser);

  frc::SmartDashboard::PutNumber("CloseAngle", 49.0);
  frc::SmartDashboard::PutNumber("ShootDelay", m_shootDelayMs);
  frc::SmartDashboard::PutNumber("AmpShotTurns", 21);
  frc::SmartDashboard::PutNumber("AmpIntakePercent", 0.0);

  // for (int moduleNumber = 0; moduleNumber < 4; moduleNumber++)
  // {
  //   m_orchestra.AddInstrument(GetDrive().GetTalon(moduleNumber));
  // }
}

//#define USE_PATH_PLANNER_SWERVE_CMD
#ifdef USE_PATH_PLANNER_SWERVE_CMD
Command* RobotContainer::GetAutonomousCommand()
{
  auto pptraj = PathPlanner::loadPath("TestPath1", units::meters_per_second_t{1.0}, units::meters_per_second_squared_t{1.0});
  // Trajectory trajectory = convertPathToTrajectory(pptraj);
  PrintTrajectory(trajectory);
  
  return GetSwerveCommandPath(trajectory);
  //return GetPathPlannerSwervePath(trajectory);
}
#else
CommandPtr RobotContainer::GetAutonomousCommand()
{
  auto autoPath = m_chooser.GetSelected();
  auto autoFile = m_pathPlannerLUT[autoPath];
  printf("loading auto path %s\n", autoFile.c_str());
  //std::shared_ptr<PathPlannerAuto> ppAuto = PathPlannerAuto::	getPathGroupFromAutoFile(autoFile);

  PIDConstants translationConstants = PIDConstants(0.5, 0.0, 0.0);
  PIDConstants rotationConstants = PIDConstants(0.5, 0.0, 0.0);
  units::meters_per_second_t maxModuleSpeed = 1_mps; //	Max speed of an individual drive module in meters/sec
  units::meter_t driveBaseRadius = 20.86_in; // Distance from the center of the robot to the farthest swerve module 
  ReplanningConfig replanningConfig;//(false, false);

  AutoBuilder::configureHolonomic(
      [this]() { return GetDrive().GetPose(); }, // Function to supply current robot pose
      [this](auto initPose) { GetDrive().ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      [this]() { return GetDrive().GetChassisSpeeds(); },
      [this](ChassisSpeeds speeds) { GetDrive().Drive(-speeds.vx, -speeds.vy, -speeds.omega, false); }, // Output function that accepts field relative ChassisSpeeds
      HolonomicPathFollowerConfig(translationConstants
                                , rotationConstants
                                , maxModuleSpeed
                                , driveBaseRadius
                                , replanningConfig ),
      [this]() { return false; }, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      &m_drive // Drive requirements, usually just a single drive subsystem
  );

  return AutoBuilder::buildAuto(autoFile);
}
#endif

void RobotContainer::Periodic()
{
  m_drive.Periodic();
  // m_vision.Periodic();
  static int count = 0;
  if (count++ % 100 == 0)
  {
    SmartDashboard::PutBoolean("FieldRelative", m_fieldRelative);
  }

static double last = 0.0;
  m_shootDelayMs = frc::SmartDashboard::GetNumber("ShootDelay", 0.5);
  if (m_shootDelayMs != last)
  {
    printf("new deley %.3f last %.3f\n", m_shootDelayMs, last);
    last = m_shootDelayMs;
  }
}

void RobotContainer::SetDefaultCommands()
{
  m_drive.SetDefaultCommand(RunCommand
  (
    [this] 
    {
      // Don't send any input if autonomous is running
      if (m_isAutoRunning == false)
      {
        // const double kDeadband = 0.02;
        const double kDeadband = 0.1;
		    const double direction = -1.0;
        const auto xInput = direction* ApplyDeadband(m_primaryController.GetLeftY(), kDeadband);
        const auto yInput = direction * ApplyDeadband(m_primaryController.GetLeftX(), kDeadband);
        const auto rotInput = ApplyDeadband(m_primaryController.GetRightX(), kDeadband);      
        const auto rotXInput = ApplyDeadband(m_primaryController.GetRightY(), kDeadband);
        const auto rotYInput = ApplyDeadband(m_primaryController.GetRightX(), kDeadband);

        const auto xSpeed = m_xspeedLimiter.Calculate(xInput) * m_drive.m_currentMaxSpeed; //kMaxSpeed;
        auto ySpeed = m_yspeedLimiter.Calculate(yInput) * m_drive.m_currentMaxSpeed; //kMaxSpeed;
        auto rot = m_rotLimiter.Calculate(rotInput) * kMaxAngularSpeed;      
        const double rotX = m_rotLimiter.Calculate(rotXInput);
        const double rotY = m_rotLimiter.Calculate(rotYInput);

        if (m_DriveStraightHook)
        {
          ySpeed = 0.0_mps;
          rot = 0.0_rad_per_s;
        }

//#define DISABLE_DRIVING
#ifndef DISABLE_DRIVING
        if (m_fieldRelative)
        {
          GetDrive().RotationDrive(xSpeed, ySpeed, rotX, rotY, m_fieldRelative);
        }
        else
        {
          GetDrive().Drive(xSpeed, ySpeed, rot, m_fieldRelative);
        }
#endif// DISABLE_DRIVING
      }
    },
    {&m_drive}
  ));
}

void RobotContainer::ConfigureBindings()
{
  ConfigPrimaryButtonBindings();
  ConfigSecondaryButtonBindings();
#ifdef USE_BUTTON_BOX
  ConfigButtonBoxBindings();
#endif
}

void RobotContainer::ConfigPrimaryButtonBindings()
{
  auto& primary = m_primaryController;

  // Primary
  // Keep the bindings in this order
  // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
#ifdef USE_TEST_BUTTONS
  // primary.A().WhileTrue(&m_wheelsBackward);
  // primary.B().WhileTrue(&m_wheelsRight);
  // primary.X().WhileTrue(&m_wheelsLeft);
  // primary.Y().WhileTrue(&m_wheelsForward);
  // primary.Y().OnTrue(&m_toggleDriveStraight);
  // primary.Start().WhileTrue(&m_OverrideOn);
  // primary.Back().WhileTrue(&m_OverrideOff);
#else

  /* -------- Drive Practice Preparation -------- */


  primary.A().WhileTrue(frc2::SequentialCommandGroup{
      PreShootCommand(*this, 129_in)
    , frc2::WaitCommand(units::time::second_t(m_shootDelayMs))
    , ShootCommand(*this)
  }.ToPtr());

  primary.B().WhileTrue(frc2::SequentialCommandGroup{
      IntakeAdjust(*this)
    , PreShootCommand(*this, 30_in)
    , frc2::WaitCommand(units::time::second_t(m_shootDelayMs))
    , ShootCommand(*this)
  }.ToPtr());
  //primary.A().WhileTrue(ShootCommand(*this).ToPtr());
  // primary.A().WhileTrue(GoToPositionCommand(*this, true).ToPtr());
  // primary.B().WhileTrue(GoToPositionCommand(*this, false).ToPtr());
  // primary.B().OnTrue(PreShootCommand(*this, 129_in, 32_deg).ToPtr());
  primary.X().OnTrue(IntakeIngest(*this).ToPtr());
  primary.Y().WhileTrue(IntakeStop(*this).ToPtr());
  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  primary.POVUp(loop).Rising().IfHigh([this] { StopAllCommand(*this).Schedule(); });
#endif
  primary.LeftBumper().OnTrue(&m_toggleFieldRelative);
  primary.RightBumper().OnTrue(&m_toggleSlowSpeed);
}

void RobotContainer::ConfigSecondaryButtonBindings()
{
  auto& secondary = m_secondaryController;

  // Keep the bindings in this order
  // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
  // secondary.A().OnTrue(PlaceOnFloor(*this).ToPtr());                          
  // secondary.B().OnTrue(PlaceLow(*this).ToPtr());                                 
  // secondary.X().OnTrue(TravelPosition(*this).ToPtr());                                
  // secondary.Y().OnTrue(PlaceHigh(*this).ToPtr());                                    

  // secondary.LeftBumper().WhileTrue(IntakeIngest(*this).ToPtr());
  // secondary.RightBumper().WhileTrue(IntakeStop(*this).ToPtr());
  // secondary.Start().WhileTrue(RotateTurntableCCW(*this).ToPtr());
  // secondary.Back().WhileTrue(RotateTurntableCW(*this).ToPtr());
  // secondary.RightTrigger().WhileTrue(RetrieveGamePiece(*this).ToPtr());

  // secondary.LeftStick().OnTrue();                            
  // secondary.RightStick().OnTrue();                           
  // secondary.LeftTrigger().WhileTrue();
  // secondary.RightTrigger().WhileTrue();

#define DRIVE_PRACTICE_SECONDARY
#ifdef DRIVE_PRACTICE_SECONDARY
  /* -------- Drive Practice Preparation -------- */

  secondary.A().OnTrue(frc2::SequentialCommandGroup{
      IntakeIngest(*this)
    , frc2::WaitCommand(0.25_s)
    // , IntakeTransfer(*this)
  }.ToPtr());                          
  secondary.B().WhileTrue(frc2::SequentialCommandGroup{
      IntakeDeploy(*this)
    , IntakeRelease(*this) 
  }.ToPtr());     
  secondary.X().OnTrue(frc2::SequentialCommandGroup{
    /*  IntakeStop(*this)
    , IntakeTransfer(*this)
    , IntakeGoToPositionCommand(*this, 9.0)
    , frc2::WaitCommand(0.4_s)
    , GoToElevationCommand(*this, 40.0_deg)
    , frc2::WaitCommand(0.4_s)
    , IntakeGoToPositionCommand(*this, 24.0)
    , frc2::WaitCommand(0.8_s)
    , GoToElevationCommand(*this, -60.0_deg)
    , frc2::WaitCommand(0.4_s)
    , IntakeGoToPositionCommand(*this, 16.0)*/
      IntakeTransfer(*this)
    , GoToElevationCommand(*this, 0.0_deg)
    , frc2::WaitCommand(0.2_s)
    , IntakeGoToPositionCommand(*this, 24.0)
    , frc2::WaitCommand(0.45_s)
    , GoToElevationCommand(*this, -60.0_deg)
    , frc2::WaitCommand(0.25_s)
    , IntakeGoToPositionCommand(*this, 16.0)
  }.ToPtr());
  secondary.Y().OnTrue(frc2::SequentialCommandGroup{
      AmpShootCommand(*this)
    , frc2::WaitCommand(0.7_s)
    /*, IntakeGoToPositionCommand(*this, 24.0)
    , frc2::WaitCommand(1.0_s)
    , GoToElevationCommand(*this, 40.0_deg)
    , frc2::WaitCommand(1.0_s)
    , IntakeGoToPositionCommand(*this, 9.0)
    , frc2::WaitCommand(1.0_s)
    , GoToElevationCommand(*this, 66.0_deg)
    , frc2::WaitCommand(1.0_s)
    , IntakeGoToPositionCommand(*this, 0.0)*/
    , IntakeGoToPositionCommand(*this, 24.0)
    , frc2::WaitCommand(0.35_s)
    , GoToElevationCommand(*this, 0.0_deg)
    , frc2::WaitCommand(0.3_s)
    , IntakeGoToPositionCommand(*this, 0.0)
    , frc2::WaitCommand(0.2_s)
    , GoToElevationCommand(*this, 33.0_deg)
  }.ToPtr());

  secondary.RightBumper().OnTrue(ParallelCommandGroup{
      IntakeTransfer(*this)
    , PreShootCommand(*this, 129_in)
  }.ToPtr());
  secondary.LeftBumper().OnTrue(ParallelCommandGroup{
      IntakeTransfer(*this)
    , PreShootCommand(*this, 30_in)
  }.ToPtr());

  secondary.Back().OnTrue(GoToElevationCommand(*this, 33.0_deg).ToPtr());

  secondary.RightTrigger(0.9).WhileTrue(ShootCommand(*this).ToPtr());

  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
//#define DASHBOARD_OVERRIDE
#ifdef DASHBOARD_OVERRIDE
  secondary.POVDown(loop).Rising().IfHigh([this] { m_goToElev.Schedule(); });
  secondary.POVRight(loop).Rising().IfHigh([this] { m_ampPositionIntake.Schedule(); });
#else
  secondary.POVUp(loop).Rising().IfHigh([this] { StopAllCommand(*this).Schedule(); });
  secondary.POVRight(loop).Rising().IfHigh([this] { IntakeRelease(*this).Schedule(); });
  secondary.POVLeft(loop).Rising().IfHigh([this] { IntakeDeploy(*this).Schedule(); });
  secondary.POVDown(loop).Rising().IfHigh([this] { IntakeStop(*this).Schedule(); });
#endif
#endif

  // secondary.X().OnTrue(&m_startOrchestra);
  // secondary.Y().OnTrue(&m_endOrchestra);

#define TEST_AMP_SHOT_WITH_INTAKE
#ifndef TEST_AMP_SHOT_WITH_INTAKE
  secondary.A().WhileTrue(frc2::SequentialCommandGroup{
      PreShootCommand(*this, 129_in)
    , frc2::WaitCommand(units::time::second_t(m_shootDelayMs))
    , ShootCommand(*this, 129_in)
  }.ToPtr());

  secondary.B().WhileTrue(frc2::SequentialCommandGroup{
      IntakeAdjust(*this)
    , PreShootCommand(*this, 30_in)
    , frc2::WaitCommand(units::time::second_t(m_shootDelayMs))
    , ShootCommand(*this, 30_in)
  }.ToPtr());
  //secondary.A().WhileTrue(ShootCommand(*this).ToPtr());
  // secondary.A().WhileTrue(GoToPositionCommand(*this, true).ToPtr());
  // secondary.B().WhileTrue(GoToPositionCommand(*this, false).ToPtr());
  secondary.B().OnTrue(PreShootCommand(*this, 129_in, 32_deg).ToPtr());
  secondary.X().OnTrue(IntakeIngest(*this).ToPtr());
  secondary.Y().WhileTrue(IntakeStop(*this).ToPtr());
  secondary.LeftBumper().WhileTrue(IntakeRelease(*this).ToPtr());
  secondary.RightBumper().OnTrue(&m_resetShooterToStart);
#else
  // secondary.B().OnTrue(frc2::SequentialCommandGroup
  //   {
  //     AmpShootCommand(*this)
  //   , frc2::WaitCommand(0.5_s)
  //   , IntakeGoToPositionCommand(*this, 24.0)
  //   , frc2::WaitCommand(0.5_s)
  //   , GoToElevationCommand(*this, 0.0_deg)
  //   , frc2::WaitCommand(0.4_s)
  //   , IntakeGoToPositionCommand(*this, 0.0)
  //   , frc2::WaitCommand(0.4_s)
  //   , GoToElevationCommand(*this, 65.9_deg)
  //   }.ToPtr());
  // secondary.RightBumper().OnTrue(AmpIntakeCommand(*this).ToPtr());
#endif
}

#ifdef USE_BUTTON_BOX
void RobotContainer::ConfigButtonBoxBindings()
{
  auto& buttonBox = m_buttonBoxController;
  // Raspberry PI Pico with gp2040 firmware Button Box
  //
  // Row	Black			    Blue			    Green				      Yellow				      Red
  // 1	  Back			    Start			    Left Stick Button	Right Stick Button	Left Bumper
  // 2	  Right Trigger	Left Trigger	X					        Y					          Right Bumper
  // 3	  B				      A				      POV Left			    POV Right			      POV Up
  //buttonBox.A().OnTrue(IntakeIngest(*this).ToPtr());                          // Blue   row 3
  //buttonBox.B().WhileTrue(IntakeRelease(*this).ToPtr());                              // Blue   row 3
  //buttonBox.Start().OnTrue(PlopperGoToPositionCommand(*this).ToPtr());                                 // Black  row 3
  //buttonBox.LeftStick().WhileTrue(PlopperShootCommand(*this).ToPtr());    
  //buttonBox.Back().OnTrue(GoToElevationCommand(*this, 66.0_deg).ToPtr());

  buttonBox.X().OnTrue(&m_goToElev);
  buttonBox.Y().OnTrue(&m_ampPositionIntake);
  buttonBox.LeftStick().OnTrue(IntakeIngest(*this).ToPtr());

  // Green  row 2
  // buttonBox.Y().OnTrue(RetrieveGamePiece(*this).ToPtr());                        // Yellow row 2

  // buttonBox.LeftBumper().OnTrue(PlaceOnFloor(*this).ToPtr());                    // Red    row 1
  // buttonBox.RightBumper().WhileTrue(IntakeRelease(*this).ToPtr());               // Red    row 2
  // buttonBox.Start().WhileTrue(&m_rotateArm);                                     // Blue   row 1
  // buttonBox.Back().WhileTrue(RotateTurntableCW(*this).ToPtr());                     // Black  row 1

  // buttonBox.LeftStick().OnTrue(&m_extendArm);                            // Green  row 1
  // buttonBox.RightStick().OnTrue(&m_retractArm);                           // Yellow row 1
  // buttonBox.LeftTrigger().WhileTrue(TravelPosition(*this).ToPtr());       // Blue   row 2
  // buttonBox.RightTrigger().WhileTrue(&m_toggleClaw);                      // Black  row 2

  // auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  // buttonBox.POVLeft(loop).Rising().IfHigh([this] { m_deployment.ExtendBackPlate(); });  // Green  row 3
  // buttonBox.POVRight(loop).Rising().IfHigh([this] { m_deployment.RetractBackPlate(); });// Yellow row 3
  // buttonBox.POVUp(loop).Rising().IfHigh([this] { PlaceHighCube(*this).Schedule(); });      // Red    row 3
  // buttonBox.POVUp(loop).Rising().IfHigh(RotateTurntableCW(*this).ToPtr()});      // Red    row 3
}
#endif

const TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};


SwerveControllerCommand<4>* RobotContainer::GetSwerveCommandPath(Trajectory trajectory)
{
  //PrintTrajectory(trajectory);

  ProfiledPIDController<units::radians> thetaController{0.01, 0.0, 0.0, kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t(-std::numbers::pi), units::radian_t(std::numbers::pi));

  SwerveControllerCommand<4>* swerveControllerCommand = new SwerveControllerCommand<4>(
      trajectory,                                                             // frc::Trajectory
      [this]() { return GetDrive().GetPose(); },                                 // std::function<frc::Pose2d()>
      m_drive.m_kinematics,                                               // frc::SwerveDriveKinematics<NumModules>
      PIDController(1.0, 0, 0.0),                // frc2::PIDController
      PIDController(1.0, 0, 0.0),                // frc2::PIDController
      thetaController,                                                        // frc::ProfiledPIDController<units::radians>
      [this](auto moduleStates) { GetDrive().SetModuleStates(moduleStates); },   // std::function< void(std::array<frc::SwerveModuleState, NumModules>)>
      {&m_drive}                                                              // std::initializer_list<Subsystem*> requirements
  );

  m_drive.SetHeading(trajectory.InitialPose().Rotation().Degrees());
  m_drive.ResetOdometry(trajectory.InitialPose());

  return swerveControllerCommand;
}

void RobotContainer::PrintTrajectory(Trajectory& trajectory)
{
  printf("Time,X,Y,HoloRot\n");
  for (auto &state:trajectory.States())
  {
      double time = state.t.to<double>();
      double x = state.pose.X().to<double>();
      double y = state.pose.Y().to<double>();
      double holoRot = state.pose.Rotation().Degrees().to<double>();
      printf("%.3f,%.3f,%.3f,%.3f\n", time, x, y, holoRot);
  }
}