#ifndef MOVE_ELEVATOR_TO_CLIMB2_POSITION_COMMAND_H
#define MOVE_ELEVATOR_TO_CLIMB2_POSITION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h"

class MoveElevatorToClimb2PositionCommand : public frc2::CommandHelper<frc2::Command, MoveElevatorToClimb2PositionCommand> {
public:
    MoveElevatorToClimb2PositionCommand(Elevator* elevatorSubsystem);

    void Initialize() override;
    bool IsFinished() override;

private:
    Elevator* m_elevatorSubsystem;
};

#endif // MOVE_ELEVATOR_TO_CLIMB2_POSITION_COMMAND_H
