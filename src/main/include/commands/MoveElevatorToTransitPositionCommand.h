#ifndef MOVE_ELEVATOR_TO_TRANSIT_POSITION_COMMAND_H
#define MOVE_ELEVATOR_TO_TRANSIT_POSITION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h"

class MoveElevatorToTransitPositionCommand : public frc2::CommandHelper<frc2::Command, MoveElevatorToTransitPositionCommand> {
public:
    MoveElevatorToTransitPositionCommand(Elevator* elevatorSubsystem);

    void Initialize() override;
    bool IsFinished() override;

private:
    Elevator* m_elevatorSubsystem;
};

#endif // MOVE_ELEVATOR_TO_TRANSIT_POSITION_COMMAND_H
