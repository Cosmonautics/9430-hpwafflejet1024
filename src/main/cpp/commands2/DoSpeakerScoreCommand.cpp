#include "commands2/DoSpeakerScoreCommand.h"

using namespace PositionConstants;

DoSpeakerScoreCommand::DoSpeakerScoreCommand(Elevator* elevatorSubsystem,
                                             Shooter* shooterSubsystem,
                                             DriveSubsystem* driveSubsystem,
                                             Limelight* limelightSubsystem) {
  AddCommands(
      MoveShooterWheelsCommand(shooterSubsystem, -1.0),
      DoAlignDriveWithAprilTagCommand(driveSubsystem, limelightSubsystem),
      DoAlignShooterWithAprilTagCommand(shooterSubsystem, limelightSubsystem),
      frc2::WaitCommand(0.5_s),
      MoveElevatorCommand(elevatorSubsystem, kElevatorShooterPosition, false),
      frc2::WaitCommand(0.5_s),
      MoveShooterFeederWheelsCommand(shooterSubsystem, -1.0),
      frc2::WaitCommand(0.5_s),
      MoveShooterFeederWheelsCommand(shooterSubsystem, 0.0),
      MoveShooterWheelsCommand(shooterSubsystem, 0.0));
}