#ifndef MOVE_TO_AMP_SPEAKER_SCORE_POSITION_COMMAND_H
#define MOVE_TO_AMP_SPEAKER_SCORE_POSITION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h"
#include "subsystems/Shooter.h"

class MoveToAmpSpeakerScorePositionCommand : public frc2::CommandHelper<frc2::Command, MoveToAmpSpeakerScorePositionCommand> {
public:
    MoveToAmpSpeakerScorePositionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Elevator* m_elevatorSubsystem;
    Shooter* m_shooterSubsystem;
};

#endif // MOVE_ELEVATOR_TO_AMP_SCORE_POSITION_COMMAND_H
