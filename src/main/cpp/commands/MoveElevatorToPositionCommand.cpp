#include "commands/MoveElevatorToPositionCommand.h"

MoveElevatorToPositionCommand::MoveElevatorToPositionCommand(Elevator& elevator, double position)
    : m_elevator(elevator), m_position(position) {}

void MoveElevatorToPositionCommand::Initialize() {
    m_elevator.MoveToPosition(m_position);
}

bool MoveElevatorToPositionCommand::IsFinished() {
    return m_elevator.AtTargetPosition();
}
