#include "commands2/base/MoveConveyorCommand.h"  // include command header

MoveConveyorCommand::MoveConveyorCommand(
    Conveyor* conveyorSubsystem,
    double speed)  // add all the relevant subsystems to constructor
    : m_conveyorSubsystem(conveyorSubsystem), m_speed(speed) {
  AddRequirements({conveyorSubsystem});
}

void MoveConveyorCommand::Initialize() {}

void MoveConveyorCommand::Execute() { m_conveyorSubsystem->Move(m_speed); }

bool MoveConveyorCommand::IsFinished() { return true; }
