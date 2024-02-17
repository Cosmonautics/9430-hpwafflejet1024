#include "commands/MoveElevatorToAMPScorePositionCommand.h"

MoveElevatorToAMPScorePositionCommand::MoveElevatorToAMPScorePositionCommand(Elevator* elevatorSubsystem)
: m_elevatorSubsystem(elevatorSubsystem) {
    AddRequirements({elevatorSubsystem});
}

void MoveElevatorToAMPScorePositionCommand::Initialize() {
    m_elevatorSubsystem->MoveToPosition(ElevatorConstants::kAMPScorePositionInches);
}

bool MoveElevatorToAMPScorePositionCommand::IsFinished() {
    return m_elevatorSubsystem->AtTargetPosition();
}
