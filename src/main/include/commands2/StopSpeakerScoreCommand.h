#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "commands2/base/StopDriveTrainCommand.h"
#include "commands2/base/MoveShooterFeederWheelsCommand.h"
#include "commands2/base/MoveShooterWheelsCommand.h"
#include "commands2/base/PivotShooterCommand.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Elevator.h"
#include "subsystems/Shooter.h"
class StopSpeakerScoreCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 StopSpeakerScoreCommand> {
 public:
  StopSpeakerScoreCommand(Shooter* shooterSubsystem,
                        DriveSubsystem* driveSubsystem);
};