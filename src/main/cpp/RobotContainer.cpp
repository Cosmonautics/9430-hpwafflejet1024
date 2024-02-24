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
  // Right Bumper
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

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
  IntakeDropNoteCommand(&m_intake, false, 0));

  // Right Bumper (Shoot)
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
      .OnTrue(new ShootMotorsCommand(m_shooter, true, 1))
      .OnFalse(new ShootMotorsCommand(m_shooter, false, 1));*/

  // Y Button (Elevator)
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
      .OnTrue(new MoveElevatorToPositionCommand(
          m_elevator, ElevatorConstants::kAMPScorePositionRotations));
  // B Button (Elevator)
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
      .OnTrue(new MoveElevatorToPositionCommand(
          m_elevator, ElevatorConstants::kTransitPositionRotations));

  // Floor Intake Position
  /*frc2::POVButton(&m_driverController, 270)
      .OnTrue(
          new GoToFloorIntakePositionCommand(m_elevator, m_shooter,
     m_intake));*/
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  choreolib::ChoreoTrajectory traj =
      choreolib::Choreo::GetTrajectory("NewPath");
  m_drive.ResetOdometry(traj.GetInitialPose());
  choreolib::ChoreoControllerFunction controller =
      choreolib::Choreo::ChoreoSwerveController(
          frc::PIDController(AutoConstants::kPXController, 0.0, 0.0),
          frc::PIDController(AutoConstants::kPYController, 0.0, 0.0),
          frc::PIDController(AutoConstants::kPThetaController, 0.0, 0.0));
  choreolib::ChoreoSwerveCommand swerveDriveCommand =
      choreolib::ChoreoSwerveCommand(
          traj, [this]() { return m_drive.GetPose(); }, controller,
          [this](auto speeds) {
            m_drive.Drive(units::meters_per_second_t{speeds.vx},
                          units::meters_per_second_t{speeds.vy}, speeds.omega,
                          false, true);
          },
          [this]() {
            return frc::DriverStation::GetAlliance() ==
                   frc::DriverStation::kRed;
          },
          {&m_drive});

  auto resetOdometry = [this, traj]() {
    m_drive.ResetOdometry(traj.GetInitialPose());
  };

  auto stopRobotDrive = [this]() {
    m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);
  };

  frc2::Command* commandGroup = new frc2::SequentialCommandGroup(
      frc2::InstantCommand(resetOdometry, {&m_drive}),
      std::move(swerveDriveCommand),
      frc2::RunCommand(stopRobotDrive, {&m_drive}));

  return commandGroup;
}
