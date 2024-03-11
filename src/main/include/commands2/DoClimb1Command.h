#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "commands2/base/MoveElevatorCommand.h"
#include "commands2/base/PivotIntakeCommand.h"
#include "commands2/base/PivotShooterCommand.h"
#include "commands2/base/SetElevatorBrakeCommand.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

class DoClimb1Command : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                                   DoClimb1Command> {
 public:
  DoClimb1Command(Elevator* elevatorSubsystem, Shooter* shooterSubsystem,
                  Intake* intakeSubsystem);
};