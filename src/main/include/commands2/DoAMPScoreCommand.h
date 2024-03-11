#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "commands2/base/MoveElevatorCommand.h"
#include "commands2/base/MoveShooterFeederWheelsCommand.h"
#include "commands2/base/MoveShooterWheelsCommand.h"
#include "commands2/base/PivotShooterCommand.h"
#include "subsystems/Elevator.h"
#include "subsystems/Shooter.h"

class DoAMPScoreCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 DoAMPScoreCommand> {
 public:
  DoAMPScoreCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem);
};