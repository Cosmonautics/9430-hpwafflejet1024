#ifndef MOVE_TO_AMP_SPEAKER_SCORE_POSITION_COMMAND_H
#define MOVE_TO_AMP_SPEAKER_SCORE_POSITION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>
#include "subsystems/Elevator.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

class MoveToAMPSpeakerScorePositionCommand : public frc2::CommandHelper<frc2::Command, MoveToAMPSpeakerScorePositionCommand> {
public:
    MoveToAMPSpeakerScorePositionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Elevator* m_elevatorSubsystem;
    Shooter* m_shooterSubsystem;
};

#endif // MOVE_TO_AMP_SPEAKER_SCORE_POSITION_COMMAND_H