// RunConveyorCommand.cpp

#include "commands/RunConveyorCommand.h"

RunConveyorCommand::RunConveyorCommand(Conveyor* subsystem, bool forward)
    : m_conveyorSubsystem(subsystem), m_forward(forward) {
  AddRequirements({subsystem});
}

void RunConveyorCommand::Initialize() {}

void RunConveyorCommand::Execute() {
  if (m_forward) {
    m_conveyorSubsystem->Forward();
  } else {
    m_conveyorSubsystem->Reverse();
  }
}

void RunConveyorCommand::End(bool interrupted) { m_conveyorSubsystem->Stop(); }

bool RunConveyorCommand::IsFinished() { return false; }
