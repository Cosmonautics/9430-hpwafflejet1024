#include "commands2/DoAMPScoreCommand.h"

using namespace PositionConstants;

DoAMPScoreCommand::DoAMPScoreCommand(Elevator* elevatorSubsystem,
                                     Shooter* shooterSubsystem) {
  AddCommands(
      MoveShooterWheelsCommand(shooterSubsystem, -1.0),
      PivotShooterCommand(shooterSubsystem, kShooterShooterPosition),
      MoveElevatorCommand(elevatorSubsystem, kElevatorShooterPosition, false),
      frc2::WaitCommand(1.5_s),
      MoveShooterFeederWheelsCommand(shooterSubsystem, -1.0),
      frc2::WaitCommand(0.5_s),
      MoveShooterFeederWheelsCommand(shooterSubsystem, 0.0),
      MoveShooterWheelsCommand(shooterSubsystem, 0.0));
}