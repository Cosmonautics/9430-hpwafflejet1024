// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include "commands2/DoSpeakerScoreCommand.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Elevator.h"
#include "subsystems/Limelight.h"
#include "subsystems/Shooter.h"

/** Container for auto command factories. */
namespace autos {
/**
 * A complex auto command that drives forward, releases a hatch, and then drives
 * backward.
 */
frc2::CommandPtr GetAndShootFirstThree(Elevator* elevatorSubsystem,
                                       Shooter* shooterSubsystem,
                                       DriveSubsystem* driveSubsystem,
                                       Limelight* limelightSubsystem);
frc2::CommandPtr ThreeNoteAuto(Elevator* elevatorSubsystem,
                               Shooter* shooterSubsystem,
                               DriveSubsystem* driveSubsystem,
                               Limelight* limelightSubsystem);
frc2::CommandPtr OneNoteAutoOnSteroids(Elevator* elevatorSubsystem,
                               Shooter* shooterSubsystem,
                               DriveSubsystem* driveSubsystem,
                               Limelight* limelightSubsystem);
}  // namespace autos