#include "commands/MoveElevatorToFloorIntakePositionCommand.h"

MoveElevatorToFloorIntakePositionCommand::MoveElevatorToFloorIntakePositionCommand(Elevator* elevatorSubsystem)
: m_elevatorSubsystem(elevatorSubsystem) {
    AddRequirements({elevatorSubsystem});
}

void MoveElevatorToFloorIntakePositionCommand::Initialize() {
    m_elevatorSubsystem->MoveToPosition(ElevatorConstants::kFloorIntakePositionInches);
}

bool MoveElevatorToFloorIntakePositionCommand::IsFinished() {
    return m_elevatorSubsystem->AtTargetPosition();
}
