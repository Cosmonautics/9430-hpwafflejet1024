#include "commands/MoveElevatorToClimb2PositionCommand.h"

MoveElevatorToClimb2PositionCommand::MoveElevatorToClimb2PositionCommand(Elevator* elevatorSubsystem)
: m_elevatorSubsystem(elevatorSubsystem) {
    AddRequirements({elevatorSubsystem});
}

void MoveElevatorToClimb2PositionCommand::Initialize() {
    m_elevatorSubsystem->MoveToPosition(ElevatorConstants::kClimb2PositionInches);
}

bool MoveElevatorToClimb2PositionCommand::IsFinished() {
    return m_elevatorSubsystem->AtTargetPosition();
}
