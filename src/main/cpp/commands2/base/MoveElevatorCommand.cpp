#include "commands2/base/MoveElevatorCommand.h"  // include command header

MoveElevatorCommand::MoveElevatorCommand(  // Find and Replace
                                           // "MoveElevatorCommand" with the
                                           // name of the command
    Elevator* elevatorSubsystem, double position,
    bool isClimb)  // add all the relevant subsystems to constructor
    : m_elevatorSubsystem(elevatorSubsystem),
      m_position(position),
      m_isClimb(isClimb) {
  AddRequirements({elevatorSubsystem});
}

void MoveElevatorCommand::Initialize() {}

void MoveElevatorCommand::Execute() {
  m_elevatorSubsystem->MoveToPosition(m_position, m_isClimb);
}

bool MoveElevatorCommand::IsFinished() { return true; }
