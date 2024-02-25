#ifndef MOVE_TO_CLIMB2_POSITION_COMMAND_H
#define MOVE_TO_CLIMB2_POSITION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h"

class MoveToClimb2PositionCommand : public frc2::CommandHelper<frc2::Command, MoveToClimb2PositionCommand> {
public:
    MoveToClimb2PositionCommand(Elevator* elevatorSubsystem);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Elevator* m_elevatorSubsystem;
};

#endif // MOVE_ELEVATOR_TO_CLIMB2_POSITION_COMMAND_H
