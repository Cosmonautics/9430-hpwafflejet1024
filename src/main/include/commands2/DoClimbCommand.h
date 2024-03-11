#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "commands2/base/MoveElevatorCommand.h"
#include "commands2/base/SetElevatorBrakeCommand.h"
#include "subsystems/Elevator.h"
#include "subsystems/Shooter.h"

class DoClimbCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, DoClimbCommand> {
 public:
  DoClimbCommand(Elevator* elevatorSubsystem);
};