#ifndef SPEAKER_SCORE_ACTION_COMMAND_H
#define SPEAKER_SCORE_ACTION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Shooter.h"

class SpeakerScoreActionCommand : public frc2::CommandHelper<frc2::Command, SpeakerScoreActionCommand> {
public:
    SpeakerScoreActionCommand(Shooter* shooterSubsystem);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Shooter* m_shooterSubsystem;
};

#endif //MOVE_ELEVATOR_TO_CLIMB_1_POSITION_COMMAND