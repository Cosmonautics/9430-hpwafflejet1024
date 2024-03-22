#include "commands2/DoSpeakerScoreManualCommand.h"

using namespace PositionConstants;

DoSpeakerScoreManualCommand::DoSpeakerScoreManualCommand(
    Elevator* elevatorSubsystem, Shooter* shooterSubsystem) {
  AddCommands(
      MoveShooterWheelsCommand(shooterSubsystem, -1.0),
      PivotShooterCommand(shooterSubsystem, 0.846), frc2::WaitCommand(0.5_s),
      MoveElevatorCommand(elevatorSubsystem, kElevatorShooterPosition, false),
      frc2::WaitCommand(0.5_s),
      MoveShooterFeederWheelsCommand(shooterSubsystem, -1.0),
      frc2::WaitCommand(0.5_s),
      MoveShooterFeederWheelsCommand(shooterSubsystem, 0.0),
      MoveShooterWheelsCommand(shooterSubsystem, 0.0));
}