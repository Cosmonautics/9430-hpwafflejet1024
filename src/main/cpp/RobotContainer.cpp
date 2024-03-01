#pragma once
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <choreo/lib/Choreo.h>
#include <choreo/lib/ChoreoSwerveCommand.h>
#include <choreo/lib/ChoreoTrajectory.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_elevator.SetDefaultCommand(frc2::RunCommand(
      [this] {
        double rightTriggerValue = m_operatorController.GetRightTriggerAxis();
        double leftTriggerValue = m_operatorController.GetLeftTriggerAxis();

        // Apply deadband to the trigger values
        rightTriggerValue = frc::ApplyDeadband(
            rightTriggerValue, ElevatorConstants::kTriggerDeadband);
        leftTriggerValue = frc::ApplyDeadband(
            leftTriggerValue, ElevatorConstants::kTriggerDeadband);

        double triggerValue = 0;

        // If right trigger is pressed more than the left, move up (positive
        // direction)
        if (rightTriggerValue > leftTriggerValue) {
          triggerValue = rightTriggerValue;  // Positive direction
        }
        // If left trigger is pressed more than the right, move down (negative
        // direction)
        else if (leftTriggerValue > rightTriggerValue) {
          triggerValue = -leftTriggerValue;  // Negative direction
        }
        // If both triggers are pressed equally or not at all, don't move
        // (triggerValue remains 0)

        // Use the triggerValue to control the elevator. Assuming SetSpeed or a
        // similar method controls the elevator's speed.
        m_elevator.ManualMove(triggerValue);
      },
      {&m_elevator}));

  m_shooter.SetDefaultCommand(frc2::RunCommand(
      [this] {
        double rightTriggerValue = m_operatorController.GetPOV(0);
        double leftTriggerValue = m_operatorController.GetPOV(4);

        // Apply deadband to the trigger values
        rightTriggerValue = rightTriggerValue;
        leftTriggerValue = leftTriggerValue;

        double triggerValue = 0;

        // If right trigger is pressed more than the left, move up (positive
        // direction)
        if (rightTriggerValue > leftTriggerValue) {
          triggerValue = rightTriggerValue;  // Positive direction
        }
        // If left trigger is pressed more than the right, move down (negative
        // direction)
        else if (leftTriggerValue > rightTriggerValue) {
          triggerValue = -leftTriggerValue;  // Negative direction
        }
        // If both triggers are pressed equally or not at all, don't move
        // (triggerValue remains 0)

        // Use the triggerValue to control the elevator. Assuming SetSpeed or a
        // similar method controls the elevator's speed.
        m_shooter.ManualMove(triggerValue);
      },
      {&m_shooter}));

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  // SPEED IS AT 50%

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(  // FLAG: x and y might be switched here.
            -units::meters_per_second_t{
                frc::ApplyDeadband(m_driverController.GetLeftY() * 0.75,
                                   OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{
                frc::ApplyDeadband(m_driverController.GetLeftX() * 0.75,
                                   OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{
                frc::ApplyDeadband(m_driverController.GetRightX() * 0.75,
                                   OIConstants::kDriveDeadband)},
            true, true);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  frc::Timer holdTimer;

  /*frc2::JoystickButton(&m_operatorController,
                       frc::XboxController::Button::kBack)
      .OnTrue(new frc2::InstantCommand(
          [this] {

          },
          {&m_elevator, &m_shooter}))*/
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart)
      .OnTrue(new frc2::InstantCommand(
          [this] {
            m_drive.ZeroHeading();
            ControllerUtils::VibrateController(m_driverController, 0.8, 0.3_s);
          },
          {&m_drive}));
  frc::ApplyDeadband(m_operatorController.GetRightTriggerAxis(),
                     ElevatorConstants::kTriggerDeadband);

  // D-Pad Left (Floor Intake)
  frc2::POVButton(&m_operatorController, 270)
      .OnTrue(new MoveToFloorIntakePositionCommand(&m_elevator, &m_shooter,
                                                   &m_intake));

  frc2::POVButton(&m_operatorController, 90)
      .OnTrue(
          new MoveToAMPSpeakerScorePositionCommand(&m_elevator, &m_shooter));

  frc2::POVButton(&m_operatorController, 180)
      .OnTrue(
          new MoveToTransitPositionCommand(&m_elevator, &m_shooter, &m_intake));

  frc2::POVButton(&m_operatorController, 0)
      .OnTrue(new frc2::InstantCommand(
          [this] {
            if (!isClimb2) {
              isClimb1 = true;
              (new MoveToClimbPos1Command(&m_elevator, &m_shooter, &m_intake))
                  ->Schedule();
            } /*else {
              isClimb1 = true;
              (new MoveToClimbPos2Command(&m_elevator))->Schedule();
            }
            isClimb2 = !isClimb2;*/
            // USE ABOVE FOR WHEN WE DO USE CLIMB 2
          },
          {&m_elevator, &m_shooter, &m_intake}));

  /*frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kX)
      .OnTrue(new DoClimbActionCommand(&m_intake, true, -0.10))
      .OnFalse(new DoClimbActionCommand(&m_intake, false, 0));*/

  frc2::JoystickButton(&m_operatorController,
                       frc::XboxController::Button::kLeftBumper)
      .OnTrue(new DoNoteIntakeActionCommand(&m_conveyor, &m_shooter, &m_intake))
      .OnFalse(new StopNoteIntakeEjectActionCommand(&m_conveyor, &m_shooter,
                                                    &m_intake));

  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kB)
      .OnTrue(new DoAMPScoreActionCommand(&m_elevator, &m_shooter));

  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kA)
      .OnTrue(new DoNoteEjectActionCommand(&m_conveyor, &m_shooter, &m_intake))
      .OnFalse(new StopNoteIntakeEjectActionCommand(&m_conveyor, &m_shooter,
                                                    &m_intake));

  frc2::JoystickButton(&m_operatorController,
                       frc::XboxController::Button::kRightBumper)
      .OnTrue(new DoSpeakerScoreActionCommand(&m_elevator, &m_shooter));

  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kY)
      .OnTrue(new frc2::InstantCommand(
          [this] {
            if (isClimb1) {
              isClimb1 = false;
              (new DoClimbActionCommand(&m_elevator, &m_shooter))->Schedule();
            }
          },
          {&m_elevator, &m_shooter}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);
  // Retrieve trajectory data from NetworkTable
  /* auto inst = nt::NetworkTableInstance::GetDefault();
   auto trajectoryEntry = inst.GetEntry("TrajectoryTablePoints");
   std::string jsonData = trajectoryEntry.GetString("");
   std::string decoded = base64_decode(jsonData);
   std::vector<frc::Pose2d> trajectoryPoints =
       ParseTrajectoryJson(nlohmann::json::parse(decoded));

   // Ensure trajectoryPoints has at least start and end points
   if (trajectoryPoints.size() < 2) {
     // Handle error: insufficient points
     return nullptr;  // or some default command
   }

   // Extract start and end pose
   frc::Pose2d startPose = trajectoryPoints.front();
   frc::Pose2d endPose = trajectoryPoints.back();

   // Extract waypoints, omitting the first and last
   std::vector<frc::Translation2d> waypoints;
   for (size_t i = 1; i < trajectoryPoints.size() - 1; i++) {
     waypoints.push_back(trajectoryPoints[i].Translation());
   }
 */
  // Generate the trajectory
  /*auto generatedTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      startPose, waypoints, endPose, config);*/
  auto generatedTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the first point
      frc::Pose2d{0.98_m, 10.06_m, 0.00_deg},
      // Pass through these waypoints
      {frc::Translation2d{1.28_m, 10.06_m},
       frc::Translation2d{1.54_m, 10.08_m},
       frc::Translation2d{1.80_m, 10.08_m},
       frc::Translation2d{2.06_m, 10.08_m},
       frc::Translation2d{2.32_m, 10.08_m},
       frc::Translation2d{2.58_m, 10.08_m},
       frc::Translation2d{2.86_m, 10.10_m},
       frc::Translation2d{3.12_m, 10.10_m},
       frc::Translation2d{3.38_m, 10.10_m},
       frc::Translation2d{3.66_m, 10.12_m},
       frc::Translation2d{3.92_m, 10.12_m},
       frc::Translation2d{4.18_m, 10.12_m},
       frc::Translation2d{4.44_m, 10.12_m},
       frc::Translation2d{4.70_m, 10.12_m},
       frc::Translation2d{4.96_m, 10.12_m},
       frc::Translation2d{5.22_m, 10.14_m},
       frc::Translation2d{5.48_m, 10.18_m},
       frc::Translation2d{5.74_m, 10.18_m},
       frc::Translation2d{6.00_m, 10.18_m},
       frc::Translation2d{6.26_m, 10.22_m},
       frc::Translation2d{6.52_m, 10.26_m},
       frc::Translation2d{6.78_m, 10.26_m}},
      // End at the last point
      frc::Pose2d{7.04_m, 10.26_m, -180.00_deg},
      // Pass the config
      config);
      
  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      generatedTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(generatedTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); },
          {}));
}
