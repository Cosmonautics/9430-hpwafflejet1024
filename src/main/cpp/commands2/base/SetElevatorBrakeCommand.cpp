#include "commands2/base/SetElevatorBrakeCommand.h"  // include command header

SetElevatorBrakeCommand::SetElevatorBrakeCommand(  // Find and Replace
                                                   // "SetElevatorBrakeCommand"
                                                   // with the name of the
                                                   // command
    Elevator* elevatorSubsystem,
    bool brake)  // add all the relevant subsystems to constructor
    : m_elevatorSubsystem(elevatorSubsystem), m_brake(brake) {
  AddRequirements({elevatorSubsystem});
}

void SetElevatorBrakeCommand::Initialize() {}

void SetElevatorBrakeCommand::Execute() {
  m_elevatorSubsystem->SetToBrakeMode();
}

bool SetElevatorBrakeCommand::IsFinished() { return true; }
