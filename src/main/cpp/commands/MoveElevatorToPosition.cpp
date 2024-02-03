#include "commands/MoveElevatorToPosition.h"

MoveElevatorToPosition::MoveElevatorToPosition(Elevator& elevator, double position)
    : m_elevator(elevator), m_position(position) {}

void MoveElevatorToPosition::Initialize() {
    m_elevator.MoveToPosition(m_position);
}

bool MoveElevatorToPosition::IsFinished() {
    return m_elevator.AtTargetPosition();
}
