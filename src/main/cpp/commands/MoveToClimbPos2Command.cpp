#include "commands/MoveToClimbPos2Command.h" 

MoveToClimbPos2Command::MoveToClimbPos2Command( 
    Elevator* elevatorSubsystem)
    : m_elevatorSubsystem(elevatorSubsystem) {
  AddRequirements({elevatorSubsystem});
}

void MoveToClimbPos2Command::Initialize() {}

void MoveToClimbPos2Command::Execute() {
  m_elevatorSubsystem->MoveToPosition(
      PositionConstants::kElevatorClimb2Position);
}

bool MoveToClimbPos2Command::IsFinished() {
    return m_elevatorSubsystem->AtTargetPosition();
}