#ifndef MOVE_ELEVATOR_TO_FLOOR_INTAKE_POSITION_COMMAND_H
#define MOVE_ELEVATOR_TO_FLOOR_INTAKE_POSITION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h"

class MoveElevatorToFloorIntakePositionCommand : public frc2::CommandHelper<frc2::Command, MoveElevatorToFloorIntakePositionCommand> {
public:
    MoveElevatorToFloorIntakePositionCommand(Elevator* elevatorSubsystem);

    void Initialize() override;
    bool IsFinished() override;

private:
    Elevator* m_elevatorSubsystem;
};

#endif // MOVE_ELEVATOR_TO_FLOOR_INTAKE_POSITION_COMMAND_H
