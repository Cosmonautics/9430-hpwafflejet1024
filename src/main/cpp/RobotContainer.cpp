#pragma once
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

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
        double rightTriggerValue = m_driverController.GetRightTriggerAxis();
        double leftTriggerValue = m_driverController.GetLeftTriggerAxis();

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

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  // SPEED IS AT 50%

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(  // FLAG: x and y might be switched here.
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true, true);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  frc::Timer holdTimer;
  // Right Bumper

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack)
      .OnTrue(new frc2::InstantCommand(
          [this, &holdTimer] {
            holdTimer.Reset();
            holdTimer.Start();
          },
          {&m_elevator}))
      .OnFalse(new frc2::InstantCommand(
          [this, &holdTimer] {
            holdTimer.Stop();
            if (holdTimer.HasElapsed(0.5_s)) {
              m_elevator.ToggleManualOverride();
              ControllerUtils::VibrateController(m_driverController, 0.8,
                                                 0.3_s);
            }
          },
          {&m_elevator}));
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart)
      .OnTrue(new frc2::InstantCommand(
          [this, &holdTimer] {
            holdTimer.Reset();
            holdTimer.Start();
          },
          {&m_drive}))
      .OnFalse(new frc2::InstantCommand(
          [this, &holdTimer] {
            holdTimer.Stop();
            if (holdTimer.HasElapsed(0.5_s)) {
              m_drive.ZeroHeading();
              ControllerUtils::VibrateController(m_driverController, 0.8,
                                                 0.3_s);
            }
          },
          {&m_drive}));
  frc::ApplyDeadband(m_driverController.GetRightTriggerAxis(),
                     ElevatorConstants::kTriggerDeadband);

  // X Button (Reload/Pickup Note)
  /*frc2::JoystickButton(&m_driverController,
  frc::XboxController::Button::kX) .OnTrue(new
  IntakePickUpNoteCommand(&m_intake, true, -0.10)) .OnFalse(new
  IntakePickUpNoteCommand(&m_intake, false, 0));

  // B Button (Drop Note)
  frc2::JoystickButton(&m_driverController,
  frc::XboxController::Button::kB) .OnTrue(new
  IntakeDropNoteCommand(&m_intake, true, 0.10)) .OnFalse(new
  IntakeDropNoteCommand(&m_intake, false, 0));*/

  // B Button (Shoot Note)
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
      .OnTrue(new ShootMotorsCommand(m_shooter, true, 1))
      .OnFalse(new ShootMotorsCommand(m_shooter, false, 0));

  // D-Pad Right (Elevator)
  frc2::POVButton(&m_driverController, 90)
      .OnTrue(
          new MoveToAmpSpeakerScorePositionCommand(&m_elevator, &m_shooter));
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
      frc::Pose2d{0.00_m, 0.00_m, 0.00_deg},
      // Pass through these waypoints
      {frc::Translation2d{12.60_m, 6.08_m}, frc::Translation2d{12.80_m, 6.10_m},
       frc::Translation2d{13.02_m, 6.10_m}, frc::Translation2d{13.22_m, 6.10_m},
       frc::Translation2d{13.42_m, 6.04_m}, frc::Translation2d{13.58_m, 5.92_m},
       frc::Translation2d{13.70_m, 5.72_m}, frc::Translation2d{13.70_m, 5.52_m},
       frc::Translation2d{13.60_m, 5.34_m}, frc::Translation2d{13.48_m, 5.16_m},
       frc::Translation2d{13.32_m, 5.04_m}, frc::Translation2d{13.12_m, 4.98_m},
       frc::Translation2d{12.90_m, 4.96_m}, frc::Translation2d{12.68_m, 4.96_m},
       frc::Translation2d{12.46_m, 4.96_m}, frc::Translation2d{12.26_m, 4.96_m},
       frc::Translation2d{12.04_m, 4.96_m}, frc::Translation2d{11.80_m, 4.96_m},
       frc::Translation2d{11.62_m, 5.06_m}, frc::Translation2d{11.44_m, 5.20_m},
       frc::Translation2d{11.30_m, 5.36_m}, frc::Translation2d{11.22_m, 5.56_m},
       frc::Translation2d{11.22_m, 5.76_m}, frc::Translation2d{11.40_m, 5.88_m},
       frc::Translation2d{11.58_m, 5.98_m}, frc::Translation2d{11.78_m, 6.04_m},
       frc::Translation2d{11.98_m, 6.10_m}},
      // End at the last point
      frc::Pose2d{0.00_m, 0.00_m, 174.29_deg},
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
