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
#include <frc/smartdashboard/SmartDashboard.h>
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
  ConfigureAutoChooser();
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
              (new DoClimb1Command(&m_elevator, &m_shooter, &m_intake))
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
      .OnTrue(new DoAMPScoreCommand(&m_elevator, &m_shooter));

  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kA)
      .OnTrue(new DoNoteEjectActionCommand(&m_conveyor, &m_shooter, &m_intake))
      .OnFalse(new StopNoteIntakeEjectActionCommand(&m_conveyor, &m_shooter,
                                                    &m_intake));
  frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kX)
      .OnTrue(new DoSourceIntakeActionCommand(&m_elevator, &m_shooter))
      .OnFalse(new StopSourceIntakeActionCommand(&m_elevator, &m_shooter));

  frc2::JoystickButton(&m_operatorController,
                       frc::XboxController::Button::kRightBumper)
      .OnTrue(new frc2::InstantCommand(
          [this] {
            frc::SmartDashboard::PutNumber("Limelight Distance", m_limelight.CalculateDistanceToTarget(true));
            (new DoSpeakerScoreCommand(&m_elevator, &m_shooter, &m_drive,
                                       &m_limelight))
                ->Schedule();
          },
          {&m_elevator, &m_shooter, &m_drive, &m_limelight}));

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

void RobotContainer::ConfigureAutoChooser() {
  m_chooser.AddOption("Do Nothing", AutonomousOption::DoNothing);
  m_chooser.AddOption("Shoot Note and Do Nothing", AutonomousOption::ShootNote);
  m_chooser.AddOption("Get First 3 Notes and Shoot",
                      AutonomousOption::GetAndShootFirstThree);
  // Add more options as needed

  frc::SmartDashboard::PutData("Autonomous Options", &m_chooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  switch (m_chooser.GetSelected()) {
    case AutonomousOption::DoNothing:
      return new frc2::InstantCommand([]() {}, {});
    case AutonomousOption::ShootNote:
      return new DoSpeakerScoreActionCommand(&m_elevator, &m_shooter);
      break;
    case AutonomousOption::GetAndShootFirstThree: {
      choreolib::ChoreoTrajectory traj =
          choreolib::Choreo::GetTrajectory("NewPath");

      m_drive.ResetOdometry(traj.GetInitialPose());

      frc::PIDController thetaController =
          frc::PIDController(AutoConstants::kPThetaController, 0.0, 0.0);

      thetaController.EnableContinuousInput(-M_PI, M_PI);

      choreolib::ChoreoControllerFunction controller =
          choreolib::Choreo::ChoreoSwerveController(
              frc::PIDController(AutoConstants::kPXController, 0.0, 0.0),
              frc::PIDController(AutoConstants::kPYController, 0.0, 0.0),
              thetaController);

      choreolib::ChoreoSwerveCommand swerveDriveCommand =
          choreolib::ChoreoSwerveCommand(
              traj, [this]() { return m_drive.GetPose(); }, controller,
              [this](frc::ChassisSpeeds speeds) {
                m_drive.Drive(units::meters_per_second_t{speeds.vx},
                              units::meters_per_second_t{speeds.vy},
                              speeds.omega, false, true);
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

      return new frc2::SequentialCommandGroup(
          DoSpeakerScoreActionCommand(&m_elevator, &m_shooter),
          frc2::InstantCommand(resetOdometry, {&m_drive}), swerveDriveCommand,
          frc2::RunCommand(stopRobotDrive, {&m_drive}));
      break;
    }
    // Add more cases as needed
    default:
      return new frc2::InstantCommand([]() {}, {});
  }
  return nullptr;  // In case no match is found, though this should not happen
}