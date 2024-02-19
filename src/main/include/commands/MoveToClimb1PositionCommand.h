#ifndef MOVE_TO_CLIMB1_POSITION_COMMAND_H
#define MOVE_TO_CLIMB1_POSITION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

class MoveToClimb1PositionCommand : public frc2::CommandHelper<frc2::Command, MoveToClimb1PositionCommand> {
public:
    MoveToClimb1PositionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem, Intake* intakeSubsystem);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Elevator* m_elevatorSubsystem;
    Shooter* m_shooterSubsystem;
    Intake* m_intakeSubsystem;
};

#endif