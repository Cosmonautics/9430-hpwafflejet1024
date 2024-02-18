// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "AHRS.h"
#include "Constants.h"
#include "commands/ClimbActionCommand.h"
#include "commands/ControlIntakeMotorsCommand.h"
#include "commands/IntakeDropNoteCommand.h"
#include "commands/IntakePickUpNoteCommand.h"
#include "commands/MoveElevatorToAMPScorePositionCommand.h"
#include "commands/MoveElevatorToClimb1PositionCommand.h"
#include "commands/MoveElevatorToClimb2PositionCommand.h"
#include "commands/MoveElevatorToFloorIntakePositionCommand.h"
#include "commands/MoveElevatorToPositionCommand.h"
#include "commands/MoveElevatorToTransitPositionCommand.h"
#include "commands/PivotIntakeToAngleCommand.h"
#include "commands/PivotShooterToAMPScorePositionCommand.h"
#include "commands/PivotShooterToClimb1PositionCommand.h"
#include "commands/PivotShooterToFloorIntakePositionCommand.h"
#include "commands/PivotShooterToTransitPositionCommand.h"
#include "commands/PivotToPositionCommand.h"
#include "commands/ShootMotorsCommand.h"
#include "subsystems/Conveyor.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

  // In RobotContainer.cpp, within the RobotContainer constructor:
  // Enum to keep track of the last pressed trigger

  // Variables to track the last pressed trigger and its value
  double lastTriggerValue = 0.0;
  // The robot's subsystems
  DriveSubsystem m_drive;
  Intake m_intake;
  Shooter m_shooter;
  Elevator m_elevator;
  // Conveyor m_conveyor;
  //  The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};
