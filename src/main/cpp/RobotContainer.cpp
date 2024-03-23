// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/GoToPositionCommand.h"
#include "commands/GoToAzimuthCommand.h"
#include "commands/IntakeStop.h"
#include "commands/IntakeRelease.h"
#include "commands/IntakeIngest.h"
#include "commands/IntakePreIngest.h"
#include "commands/IntakeDeploy.h"
#include "commands/StopAllCommand.h"
#include "commands/PreShootCommand.h"
#include "commands/ShootCommand.h"
#include "commands/AmpIntakeCommand.h"
#include "commands/GoToElevationCommand.h"
#include "commands/IntakeGoToPositionCommand.h"
#include "commands/IntakeTransfer.h"
#include "commands/AmpShootCommand.h"
#include "commands/ClimbCommand.h"
#include "commands/StartLEDCommand.h"
#include "commands/EndLEDCommand.h"

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

#ifndef THING1
constexpr double c_deployTurnsAmpClearance = 32.5;//20.0;
constexpr double c_deployTurnsAmpShoot = 28.5;//14.0;
#else
constexpr double c_deployTurnsAmpClearance = 30.0;
constexpr double c_deployTurnsAmpShoot = 16.0;
#endif
constexpr units::degree_t c_elevAngleAmpShoot = -60.0_deg;

RobotContainer::RobotContainer() 
  : m_drive()
  , m_orchestra("output.chrp")
{
  //---------------------------------------------------------
  //printf("************************Calling SilenceJoystickConnectionWarning - Wisco2024 Day 1 only REMOVE!!!!!\n");
  //DriverStation::SilenceJoystickConnectionWarning(true);
  //---------------------------------------------------------

  // NamedCommands::registerCommand("PreShootClose", std::move(PreShootCommand(*this, 1_m).ToPtr()));

  NamedCommands::registerCommand("ShootClose", std::move(
    frc2::SequentialCommandGroup{
        PreShootCommand(*this, 1_m)
      , frc2::WaitCommand(0.5_s)
      , ShootCommand(*this, true)
    }.ToPtr()));

  NamedCommands::registerCommand("ShootStop", std::move(StopAllCommand(*this).ToPtr()));

  NamedCommands::registerCommand("ShootFar", std::move(
    frc2::SequentialCommandGroup{
        PreShootCommand(*this, 5_m)
      , frc2::WaitCommand(0.7_s)  // Wait for elev angle to sync to gyro
      , ShootCommand(*this, true)
    }.ToPtr()));

  NamedCommands::registerCommand("Pre Ingest", std::move(IntakePreIngest(*this).ToPtr()));

  NamedCommands::registerCommand("Intake Note", std::move(IntakeIngest(*this).ToPtr()));

  SetDefaultCommands();
  ConfigureBindings();

  m_chooser.SetDefaultOption(m_pathPlannerLUT[kAutoPathDefault], EAutoPath::kAutoPathDefault);
  for (int i = 1; i < (int)m_pathPlannerLUT.size(); i++)
  {
    m_chooser.AddOption(m_pathPlannerLUT[i], (EAutoPath)i);
  }
  frc::SmartDashboard::PutData("Auto Path", &m_chooser);

  frc::SmartDashboard::PutNumber("AmpShotTurns", 21);
  frc::SmartDashboard::PutNumber("AmpIntakePercent", 0.0);

  // for (int moduleNumber = 0; moduleNumber < 4; moduleNumber++)
  // {
  //   m_orchestra.AddInstrument(GetDrive().GetTalon(moduleNumber));
  // }
}

CommandPtr RobotContainer::GetAutonomousCommand()
{
  auto autoPath = m_chooser.GetSelected();
  auto autoFile = m_pathPlannerLUT[autoPath];
  printf("loading auto path %s\n", autoFile.c_str());

  PIDConstants translationConstants = PIDConstants(0.005, 0.0, 0.0);
  PIDConstants rotationConstants = PIDConstants(1, 0, 0.025);// 0.5, 0.0, 0.0);
  units::meters_per_second_t maxModuleSpeed = 1_mps; //	Max speed of an individual drive module in meters/sec
  const units::meter_t driveBaseRadius = 17.25_in; // Distance from the center of the robot to the farthest swerve module 
  ReplanningConfig replanningConfig(false, false);

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
      [this]() 
      {
        auto alliance = DriverStation::GetAlliance();
        if (alliance)
        {
          return alliance.value() == DriverStation::Alliance::kRed;
        }
        return false; 
      }, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      &m_drive // Drive requirements, usually just a single drive subsystem
  );

  return AutoBuilder::buildAuto(autoFile);
}

void RobotContainer::Periodic()
{
  m_drive.Periodic();
  double adjustment = frc::SmartDashboard::GetNumber("SteerAdjustment", 0.0);
  double rotationInput = -1.0 * sin(adjustment);
  auto adjRot = m_yawRotationLimiter.Calculate(rotationInput) * units::radians_per_second_t{2.0};
  frc::SmartDashboard::PutNumber("AdjustRotation", adjRot.value());
  // m_vision.Periodic();
  static int count = 0;
  if (count++ % 25 == 0)
  {
    RobotContainer::ConfigureRobotLEDs();
    SmartDashboard::PutBoolean("FieldRelative", m_fieldRelative);
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

  // primary.A().WhileTrue(frc2::SequentialCommandGroup{
  //     PreShootCommand(*this, 129_in)
  //   , frc2::WaitCommand(units::time::second_t(m_shootDelayMs))
  //   , ShootCommand(*this)
  // }.ToPtr());

  // primary.B().WhileTrue(frc2::SequentialCommandGroup{
  //     PreShootCommand(*this, 30_in)
  //   , frc2::WaitCommand(units::time::second_t(m_shootDelayMs))
  //   , ShootCommand(*this)
  // }.ToPtr());

  primary.X().OnTrue(IntakeIngest(*this).ToPtr());
  primary.Y().WhileTrue(IntakeStop(*this).ToPtr());
  primary.Back().OnTrue(ClimbCommand(*this, ClimberSubsystem::kParkPosition).ToPtr());
  // primary.Back().OnTrue(frc2::SequentialCommandGroup{
  //     ClimbCommand(*this, ClimberSubsystem::kParkPosition)
  //   , frc2::WaitCommand(0.5_s)
  //   , GoToElevationCommand(*this, 74.0_deg)
  // }.ToPtr());
  
  primary.Start().OnTrue(ClimbCommand(*this, ClimberSubsystem::kHighPosition).ToPtr());
  // primary.Back().WhileTrue(&m_moveClimbDown);
  primary.LeftBumper().WhileTrue(&m_stopClimb);

  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  primary.POVUp(loop).Rising().IfHigh([this] { StopAllCommand(*this).Schedule(); });

  primary.LeftStick().OnTrue(&m_enableGyroSync);
  primary.RightStick().OnTrue(frc2::SequentialCommandGroup{
      IntakeGoToPositionCommand(*this, c_defaultRetractTurns)
    , frc2::WaitCommand(0.35_s)
    , GoToElevationCommand(*this, c_defaultTravelPosition)
  }.ToPtr());

  primary.LeftTrigger(0.9).OnTrue(frc2::SequentialCommandGroup{
      IntakeGoToPositionCommand(*this, 0.0)
    , frc2::WaitCommand(0.35_s)
    , GoToElevationCommand(*this, c_defaultStartPosition)
  }.ToPtr());

  // primary.LeftBumper().OnTrue(&m_toggleFieldRelative);
  primary.RightBumper().OnTrue(&m_toggleSlowSpeed);
}

void RobotContainer::ConfigSecondaryButtonBindings()
{
  auto& secondary = m_secondaryController;

  // Keep the bindings in this order
  // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
  secondary.A().OnTrue(frc2::SequentialCommandGroup{
      IntakeIngest(*this)
  }.ToPtr());                          
  secondary.B().WhileTrue(frc2::SequentialCommandGroup{
      IntakeRelease(*this) 
  ,   IntakeStop(*this)
  }.ToPtr());     
  secondary.X().OnTrue(frc2::SequentialCommandGroup{
      IntakeGoToPositionCommand(*this, c_deployTurnsAmpClearance)
    , StartLEDCommand(*this)
    , frc2::WaitCommand(0.15_s)
    , GoToElevationCommand(*this, c_elevAngleAmpShoot)
    , frc2::WaitCommand(0.65_s)
    , IntakeGoToPositionCommand(*this, c_deployTurnsAmpShoot)
  }.ToPtr());
  secondary.Y().OnTrue(frc2::SequentialCommandGroup{
      AmpShootCommand(*this)
    , frc2::WaitCommand(0.7_s)
    , IntakeGoToPositionCommand(*this, c_deployTurnsAmpClearance)
    , frc2::WaitCommand(0.15_s)
    , GoToElevationCommand(*this, c_defaultTravelPosition)
    , frc2::WaitCommand(0.35_s)
    , IntakeGoToPositionCommand(*this, c_defaultRetractTurns)
  }.ToPtr());

  secondary.RightBumper().OnTrue(PreShootCommand(*this, 129_in).ToPtr());
  secondary.LeftBumper().OnTrue(PreShootCommand(*this, 30_in).ToPtr());

  secondary.LeftStick().OnTrue(&m_enableGyroSync);
  secondary.RightStick().OnTrue(frc2::SequentialCommandGroup{
      IntakeGoToPositionCommand(*this, c_defaultRetractTurns)
    , frc2::WaitCommand(0.35_s)
    , GoToElevationCommand(*this, c_defaultTravelPosition)
  }.ToPtr());

  secondary.RightTrigger(0.9).WhileTrue(ShootCommand(*this).ToPtr());
  secondary.LeftTrigger(0.9).OnTrue(frc2::SequentialCommandGroup{
      IntakeGoToPositionCommand(*this, 0.0)
    , frc2::WaitCommand(0.35_s)
    , GoToElevationCommand(*this, c_defaultStartPosition)
  }.ToPtr());

  secondary.Back().OnTrue(ClimbCommand(*this, ClimberSubsystem::kParkPosition).ToPtr());
  // secondary.Back().OnTrue(frc2::SequentialCommandGroup{
  //     ClimbCommand(*this, ClimberSubsystem::kParkPosition)
  //   , frc2::WaitCommand(0.5_s)
  //   , GoToElevationCommand(*this, 74.0_deg)
  // }.ToPtr());
  secondary.Start().OnTrue(ClimbCommand(*this, ClimberSubsystem::kHighPosition).ToPtr());

  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  secondary.POVUp(loop).Rising().IfHigh([this] { StopAllCommand(*this).Schedule(); });
  secondary.POVRight(loop).Rising().IfHigh([this] { IntakeRelease(*this).Schedule(); });
  secondary.POVLeft(loop).Rising().IfHigh([this] { IntakeDeploy(*this).Schedule(); });
  secondary.POVDown(loop).Rising().IfHigh([this] { IntakeStop(*this).Schedule(); });
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
  //-------------------------------------------------------------------------------------
  // Row 1 L-R
  buttonBox.Back().OnTrue(GoToElevationCommand(*this, c_defaultStartPosition).ToPtr());      // Black
  buttonBox.Start().OnTrue(GoToElevationCommand(*this, c_defaultShootCloseAngle).ToPtr());   // Blue
  // buttonBox.LeftStick().OnTrue(GoToElevationCommand(*this, c_defaultShootFarAngle).ToPtr()); // Green
  buttonBox.LeftStick().WhileTrue(GoToAzimuthCommand(*this).ToPtr()); // Green
  buttonBox.RightStick().OnTrue(GoToElevationCommand(*this, 0.0_deg).ToPtr());               // Yellow
  buttonBox.LeftBumper().OnTrue(frc2::SequentialCommandGroup{                                // Red
      GoToElevationCommand(*this, c_defaultTravelPosition)
    , IntakeGoToPositionCommand(*this, c_deployTurnsAmpClearance)
    , StartLEDCommand(*this)
    , frc2::WaitCommand(0.15_s)
    , GoToElevationCommand(*this, c_elevAngleAmpShoot)
    , frc2::WaitCommand(0.65_s)
    , IntakeGoToPositionCommand(*this, c_deployTurnsAmpShoot)
  }.ToPtr());  

  // Row 2 L-R
  buttonBox.RightTrigger().OnTrue(GoToElevationCommand(*this, c_defaultTravelPosition).ToPtr());  // Black
  buttonBox.LeftTrigger().OnTrue(frc2::SequentialCommandGroup{                                    // Blue
      GoToElevationCommand(*this, c_defaultTravelPosition)
    , ClimbCommand(*this, ClimberSubsystem::kParkPosition)
    , frc2::WaitCommand(0.5_s)
    , GoToElevationCommand(*this, 74.0_deg)
  }.ToPtr());
  buttonBox.X().OnTrue(&m_goToElev);                                                              // Green
  buttonBox.Y().OnTrue(&m_ampPositionIntake);                                                     // Yellow
  buttonBox.RightBumper().OnTrue(frc2::SequentialCommandGroup{                                    // Red
      AmpShootCommand(*this)
    , frc2::WaitCommand(0.7_s)
    , IntakeGoToPositionCommand(*this, c_deployTurnsAmpClearance)
    , frc2::WaitCommand(0.15_s)
    , GoToElevationCommand(*this, c_defaultTravelPosition)
    , frc2::WaitCommand(0.35_s)
    , IntakeGoToPositionCommand(*this, c_defaultRetractTurns)
  }.ToPtr());
  
  // Row 3 L-R
  auto loop = CommandScheduler::GetInstance().GetDefaultButtonLoop();
  //buttonBox.B().OnTrue(StopAllCommand(*this).ToPtr());                                            // Black
  buttonBox.A().OnTrue(ClimbCommand(*this, ClimberSubsystem::kHighPosition).ToPtr());             // Blue
  buttonBox.POVLeft(loop).Rising().IfHigh([this] { m_wheelsLeft.Schedule(); });                   // Green
  buttonBox.POVRight(loop).Rising().IfHigh([this] { m_wheelsForward.Schedule(); });               // Yellow
  buttonBox.POVUp(loop).Rising().IfHigh([this] { StopAllCommand(*this).Schedule(); });            // Red

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

void RobotContainer::ConfigureRobotLEDs()
{
  bool robotEnabled = frc::SmartDashboard::GetBoolean("Robot Enabled", false);
  if (robotEnabled)
  {
    GetLED().SetDefaultColor(GetIntake().IsNotePresent() ? c_colorPink : c_colorGreen);
    if (!GetLED().IsRobotBusy())
    {
      GetLED().SetAnimation(GetLED().GetDefaultColor(), LEDSubsystem::kSolid);
    }
  }
  else
  {
    GetLED().SetAnimation(c_colorOrange, LEDSubsystem::kSolid);
  }
}