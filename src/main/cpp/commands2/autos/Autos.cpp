// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands2/autos/Autos.h"

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>

#include "Constants.h"

using namespace AutoConstants;

frc2::CommandPtr autos::GetAndShootFirstThree(Elevator* elevatorSubsystem,
                                              Shooter* shooterSubsystem,
                                              DriveSubsystem* driveSubsystem,
                                              Limelight* limelightSubsystem) {
  return frc2::cmd::Sequence(
      DoSpeakerScoreCommand(elevatorSubsystem, shooterSubsystem, driveSubsystem,
                            limelightSubsystem)
          .ToPtr(),
      frc2::WaitCommand(1.2_s).ToPtr(),
      frc2::InstantCommand(
          [driveSubsystem]() {
            driveSubsystem->ResetOdometry(
                pathplanner::PathPlannerAuto::getStartingPoseFromAutoFile(
                    "GetAndShootFirstThree"));
          },
          {})
          .ToPtr(),
      pathplanner::PathPlannerAuto("GetAndShootFirstThree").ToPtr());
}
frc2::CommandPtr autos::ThreeNoteAuto(Elevator* elevatorSubsystem,
                                      Shooter* shooterSubsystem,
                                      DriveSubsystem* driveSubsystem,
                                      Limelight* limelightSubsystem) {
  return frc2::cmd::Sequence(
      DoSpeakerScoreCommand(elevatorSubsystem, shooterSubsystem, driveSubsystem,
                            limelightSubsystem)
          .ToPtr(),
      frc2::WaitCommand(1.2_s).ToPtr(),
      frc2::InstantCommand(
          [driveSubsystem]() {
            driveSubsystem->ResetOdometry(
                pathplanner::PathPlannerAuto::getStartingPoseFromAutoFile(
                    "ThreeNoteAuto"));
          },
          {})
          .ToPtr(),
      pathplanner::PathPlannerAuto("ThreeNoteAuto").ToPtr(),
      DoSpeakerScoreCommand(elevatorSubsystem, shooterSubsystem, driveSubsystem,
                            limelightSubsystem)
          .ToPtr());
}