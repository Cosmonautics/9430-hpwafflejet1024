#include "commands2/StopSpeakerScoreCommand.h"

using namespace PositionConstants;

StopSpeakerScoreCommand::StopSpeakerScoreCommand(
    Shooter* shooterSubsystem, DriveSubsystem* driveSubsystem) {
  AddCommands(MoveShooterWheelsCommand(shooterSubsystem, 0.0),
              MoveShooterWheelsCommand(shooterSubsystem, 0.0),
              StopDriveTrainCommand(driveSubsystem));
}