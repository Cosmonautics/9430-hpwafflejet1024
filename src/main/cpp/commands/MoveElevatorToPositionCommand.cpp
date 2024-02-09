#include "commands/MoveElevatorToPositionCommand.h"

MoveElevatorToPositionCommand::MoveElevatorToPositionCommand(Elevator& elevator,
                                                             double position)
    : m_elevator(elevator), m_position(position) {}

void MoveElevatorToPositionCommand::Initialize() {}

void MoveElevatorToPositionCommand::Execute() {
  m_elevator.MoveToPosition(m_position);
}

bool MoveElevatorToPositionCommand::IsFinished() {
  bool flag = m_elevator.AtTargetPosition();
  return flag;
}
