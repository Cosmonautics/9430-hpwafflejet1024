#include "commands/MoveElevatorToClimb1PositionCommand.h"

MoveElevatorToClimb1PositionCommand::MoveElevatorToClimb1PositionCommand(Elevator* elevatorSubsystem)
: m_elevatorSubsystem(elevatorSubsystem) {
    AddRequirements({elevatorSubsystem});
}

void MoveElevatorToClimb1PositionCommand::Initialize() {
    m_elevatorSubsystem->MoveToPosition(ElevatorConstants::kClimb1PositionInches);
}

bool MoveElevatorToClimb1PositionCommand::IsFinished() {
    return m_elevatorSubsystem->AtTargetPosition();
}
