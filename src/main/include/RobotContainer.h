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
// #include "commands/CommandTemplate.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include "commands/DoAMPScoreActionCommand.h"
#include "commands/DoClimbActionCommand.h"
#include "commands/DoNoteEjectActionCommand.h"
#include "commands/DoNoteIntakeActionCommand.h"
#include "commands/DoSourceIntakeActionCommand.h"
#include "commands/DoSpeakerScoreActionCommand.h"
#include "commands/MoveToAMPSpeakerScorePositionCommand.h"
#include "commands/MoveToClimbPos1Command.h"
#include "commands/MoveToClimbPos2Command.h"
#include "commands/MoveToFloorIntakePositionCommand.h"
#include "commands/MoveToTransitPositionCommand.h"
#include "commands/StopIntakeMotorCommand.h"
#include "commands/StopNoteIntakeEjectActionCommand.h"
#include "commands/StopShooterMotorCommand.h"
#include "commands/StopSourceIntakeActionCommand.h"
#include "commands2/DoAMPScoreCommand.h"
#include "commands2/DoClimb1Command.h"
#include "commands2/DoClimbCommand.h"
#include "commands2/DoSpeakerScoreCommand.h"
#include "subsystems/Conveyor.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include "subsystems/Limelight.h"
#include "subsystems/Shooter.h"
#include "utils/ControllerUtils.h"
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

enum class AutonomousOption {
  DoNothing,
  ShootNote,
  GetAndShootFirstThree,
};

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

  // The driver's controller
  frc::XboxController m_operatorController{
      OIConstants::kOperatorControllerPort};
  // In RobotContainer.cpp, within the RobotContainer constructor:
  // Enum to keep track of the last pressed trigger

  // Variables to track the last pressed trigger and its value
  double lastTriggerValue = 0.0;
  // The robot's subsystems
  DriveSubsystem m_drive;
  Intake m_intake;
  Shooter m_shooter;
  Elevator m_elevator;
  Conveyor m_conveyor;
  Limelight m_limelight;
  // Conveyor m_conveyor;
  //  The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
  void ConfigureAutoChooser();
  void ConfigureNamedCommands();

  bool isClimb2 = false;
  bool isClimb1 = false;
};
