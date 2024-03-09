#include "commands2/DoSpeakerScoreCommand.h"

using namespace PositionConstants;

DoSpeakerScoreCommand::DoSpeakerScoreCommand(Elevator* elevatorSubsystem,
                                     Shooter* shooterSubsystem) {
  AddCommands(
      PivotShooterCommand(shooterSubsystem, kShooterAMPPosition),
      MoveElevatorCommand(elevatorSubsystem, kElevatorAMPPosition, false),
      frc2::WaitCommand(1_s),
      MoveShooterFeederWheelsCommand(shooterSubsystem, -1.0),
      MoveShooterWheelsCommand(shooterSubsystem, -0.10),
      frc2::WaitCommand(1.7_s),
      MoveShooterFeederWheelsCommand(shooterSubsystem, 0.0),
      MoveShooterWheelsCommand(shooterSubsystem, 0.0));
}