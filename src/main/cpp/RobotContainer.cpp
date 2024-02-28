// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/GoToPositionCommand.h"
#include "commands/IntakeStop.h"
#include "commands/IntakeRelease.h"
#include "commands/IntakeAdjust.h"
#include "commands/IntakeIngest.h"
#include "commands/StopAllCommand.h"
#include "commands/PreShootCommand.h"
#include "commands/ShootCommand.h"
#include "commands/PlopperGoToPositionCommand.h"
#include "commands/PlopperShootCommand.h"
#include "commands/AmpIntakeCommand.h"
#include "commands/GoToElevationCommand.h"
#include "commands/IntakeGoToPositionCommand.h"
#include "commands/IntakeTransfer.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/DriverStation.h>

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
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


using namespace pathplanner;

RobotContainer::RobotContainer() 
  : m_drive()
{
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
  auto pathFile = m_pathPlannerLUT[autoPath];
  std::shared_ptr<PathPlannerPath> path = PathPlannerPath::fromPathFile(pathFile);

  static std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;
  //eventMap.emplace("Shoot", std::make_shared<Shoot>(m_drive, *this));
  //eventMap.emplace("Intake Note", std::make_shared<Intake>(m_drive, *this));

  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this could be in RobotContainer along with your subsystems
  static AutoBuilder autoBuilder;
  
  PIDConstants translationConstants = PIDConstants(5.0, 0.0, 0.0);
  PIDConstants rotationConstants = PIDConstants(0.5, 0.0, 0.0);
  units::meters_per_second_t maxModuleSpeed = 2_mps; //	Max speed of an individual drive module in meters/sec
  units::meter_t driveBaseRadius = 20.86_in; // Distance from the center of the robot to the farthest swerve module 
  ReplanningConfig replanningConfig;

  autoBuilder.configureHolonomic(
      [this]() { return GetDrive().GetPose(); }, // Function to supply current robot pose
      [this](auto initPose) { GetDrive().ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      [this]() { return GetDrive().GetChassisSpeeds(); },
      [this](ChassisSpeeds speeds) { GetDrive().Drive(speeds.vx, speeds.vy, speeds.omega, true); }, // Output function that accepts field relative ChassisSpeeds
      HolonomicPathFollowerConfig( translationConstants, rotationConstants, maxModuleSpeed, driveBaseRadius, replanningConfig ),
      [this]() { return true; }, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      &m_drive // Drive requirements, usually just a single drive subsystem
  );

  return AutoBuilder::followPath(path);
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
#ifdef OVERUNDER
		double direction = 1.0;
#else
		double direction = -1.0;
#endif
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

  primary.A().WhileTrue(frc2::SequentialCommandGroup{
      PreShootCommand(*this, 129_in)
    , frc2::WaitCommand(units::time::second_t(m_shootDelayMs))
    , ShootCommand(*this, 129_in)
  }.ToPtr());

  primary.B().WhileTrue(frc2::SequentialCommandGroup{
      IntakeAdjust(*this)
    , PreShootCommand(*this, 30_in)
    , frc2::WaitCommand(units::time::second_t(m_shootDelayMs))
    , ShootCommand(*this, 30_in)
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

    secondary.A().OnTrue(frc2::SequentialCommandGroup{
      IntakeStop(*this)
    , frc2::WaitCommand(0.5_s)
    , GoToElevationCommand(*this, 0.0_deg)
    , frc2::WaitCommand(1.0_s)
    , IntakeGoToPositionCommand(*this, 24.0)
    , frc2::WaitCommand(1.0_s)
    , GoToElevationCommand(*this, -60.0_deg)
    , frc2::WaitCommand(1.0_s)
    , IntakeGoToPositionCommand(*this, 16.0)
    , frc2::WaitCommand(1.0_s)
    , IntakeTransfer(*this)
  }.ToPtr());
  secondary.X().OnTrue(IntakeTransfer(*this).ToPtr());
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
  // secondary.B().OnTrue(PreShootCommand(*this, 129_in, 32_deg).ToPtr());
  secondary.X().OnTrue(IntakeIngest(*this).ToPtr());
  secondary.Y().WhileTrue(IntakeStop(*this).ToPtr());
  secondary.LeftBumper().WhileTrue(IntakeRelease(*this).ToPtr());
  secondary.RightBumper().OnTrue(&m_resetShooterToStart);
#else
  secondary.B().OnTrue(&m_ampPositionIntake);
  secondary.RightBumper().OnTrue(AmpIntakeCommand(*this).ToPtr());
#endif

  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  secondary.POVUp(loop).Rising().IfHigh([this] { StopAllCommand(*this).Schedule(); });
  secondary.POVDown(loop).Rising().IfHigh([this] { m_goToElev.Schedule(); });
  //secondary.POVLeft(loop).Rising().IfHigh([this] { AmpIntakeCommand(*this).Schedule(); });
  secondary.POVRight(loop).Rising().IfHigh([this] { m_ampShootIntake.Schedule(); });
}

void RobotContainer::ConfigButtonBoxBindings()
{
  auto& buttonBox = m_buttonBoxController;
  // Raspberry PI Pico with gp2040 firmware Button Box
  //
  // Row	Black			    Blue			    Green				      Yellow				      Red
  // 1	  Back			    Start			    Left Stick Button	Right Stick Button	Left Bumper
  // 2	  Right Trigger	Left Trigger	X					        Y					          Right Bumper
  // 3	  B				      A				      POV Left			    POV Right			      POV Up
  // buttonBox.A().WhileTrue(IntakeIngest(*this).ToPtr());                          // Blue   row 3
  // buttonBox.A().OnFalse(IntakeStop(*this).ToPtr());                              // Blue   row 3
  buttonBox.Start().OnTrue(PlopperGoToPositionCommand(*this).ToPtr());                                 // Black  row 3
  buttonBox.LeftStick().WhileTrue(PlopperShootCommand(*this).ToPtr());                                // Green  row 2
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