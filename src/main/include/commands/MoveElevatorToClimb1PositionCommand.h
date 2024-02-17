#ifndef MOVE_ELEVATOR_TO_CLIMB1_POSITION_COMMAND_H
#define MOVE_ELEVATOR_TO_CLIMB1_POSITION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h"

class MoveElevatorToClimb1PositionCommand : public frc2::CommandHelper<frc2::Command, MoveElevatorToClimb1PositionCommand> {
public:
    MoveElevatorToClimb1PositionCommand(Elevator* elevatorSubsystem);

    void Initialize() override;
    bool IsFinished() override;

private:
    Elevator* m_elevatorSubsystem;
};

#endif // MOVE_ELEVATOR_TO_CLIMB1_POSITION_COMMAND_H
