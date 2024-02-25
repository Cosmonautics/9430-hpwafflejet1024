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
#include "commands/MoveElevatorToPositionCommand.h"
#include "commands/MoveToAmpSpeakerScorePositionCommand.h"
#include "commands/MoveToClimb1PositionCommand.h"
#include "commands/MoveToClimb2PositionCommand.h"
#include "commands/MoveToFloorIntakePositionCommand.h"
#include "commands/MoveToTransitPositionCommand.h"
#include "commands/NoteEjectActionCommand.h"
#include "commands/NoteIntakeActionCommand.h"
#include "commands/PivotIntakeToAngleCommand.h"
#include "commands/PivotShooterToPositionCommand.h"
#include "commands/PivotToPositionCommand.h"
#include "commands/RunConveyorCommand.h"
#include "commands/ShootMotorsCommand.h"
#include "commands/SpeakerScoreActionCommand.h"

#include "subsystems/Conveyor.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "utils/ControllerUtils.h"

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

  // The driver's controller
  frc::XboxController m_operatorController{OIConstants::kOperatorControllerPort};
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
