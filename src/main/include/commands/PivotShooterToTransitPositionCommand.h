#ifndef PIVOT_SHOOTER_TO_TRANSIT_POSITION_COMMAND_H
#define PIVOT_SHOOTER_TO_TRANSIT_POSITION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Shooter.h"

class PivotShooterToTransitPositionCommand : public frc2::CommandHelper<frc2::Command, PivotShooterToTransitPositionCommand> {
public:
    PivotShooterToTransitPositionCommand(Shooter* shooterSubsystem);

    void Initialize() override;
    bool IsFinished() override;

private:
    Shooter* m_shooterSubsystem;
};

#endif // PIVOT_SHOOTER_TO_TRANSIT_POSITION_COMMAND_H
