#ifndef PIVOT_SHOOTER_TO_AMP_SCORE_POSITION_COMMAND_H
#define PIVOT_SHOOTER_TO_AMP_SCORE_POSITION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Shooter.h"

class PivotShooterToAMPScorePositionCommand : public frc2::CommandHelper<frc2::Command, PivotShooterToAMPScorePositionCommand> {
public:
    PivotShooterToAMPScorePositionCommand(Shooter* shooterSubsystem);

    void Initialize() override;
    bool IsFinished() override;

private:
    Shooter* m_shooterSubsystem;
};

#endif // PIVOT_SHOOTER_TO_AMP_SCORE_POSITION_COMMAND_H
