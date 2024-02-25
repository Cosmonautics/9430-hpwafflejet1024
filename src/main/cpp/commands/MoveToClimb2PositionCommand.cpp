#include "commands/MoveToClimb2PositionCommand.h"

MoveToClimb2PositionCommand::MoveToClimb2PositionCommand(Elevator* elevatorSubsystem)
: m_elevatorSubsystem(elevatorSubsystem) {
    AddRequirements({elevatorSubsystem});
}

void MoveToClimb2PositionCommand::Initialize() {
    m_elevatorSubsystem->MoveToPosition(ElevatorConstants::kClimb2PositionInches);
}

void MoveToClimb2PositionCommand::Execute() {
    
}

bool MoveToClimb2PositionCommand::IsFinished() {
    return m_elevatorSubsystem->AtTargetPosition();
}
