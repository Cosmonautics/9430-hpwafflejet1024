#include "commands/MoveElevatorToTransitPositionCommand.h"

MoveElevatorToTransitPositionCommand::MoveElevatorToTransitPositionCommand(Elevator* elevatorSubsystem)
: m_elevatorSubsystem(elevatorSubsystem) {
    AddRequirements({elevatorSubsystem});
}

void MoveElevatorToTransitPositionCommand::Initialize() {
    m_elevatorSubsystem->MoveToPosition(ElevatorConstants::kTransitPositionInches);
}

bool MoveElevatorToTransitPositionCommand::IsFinished() {
    return m_elevatorSubsystem->AtTargetPosition();
}
