#include "commands2/DoSpeakerScoreAutoCommand.h"

using namespace PositionConstants;

DoSpeakerScoreAutoCommand::DoSpeakerScoreAutoCommand(
    Elevator* elevatorSubsystem, Shooter* shooterSubsystem,
    Limelight* limelightSubsystem) {
  AddCommands(
      MoveShooterWheelsCommand(shooterSubsystem, -1.0),
      frc2::WaitCommand(0.5_s),
      MoveElevatorCommand(elevatorSubsystem, kElevatorShooterPosition, false),
      frc2::WaitCommand(0.5_s),
      MoveShooterFeederWheelsCommand(shooterSubsystem, -1.0),
      frc2::WaitCommand(0.5_s),
      MoveShooterFeederWheelsCommand(shooterSubsystem, 0.0),
      MoveShooterWheelsCommand(shooterSubsystem, 0.0));
}