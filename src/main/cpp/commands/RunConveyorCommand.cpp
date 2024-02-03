// RunConveyorCommand.cpp

#include "commands/RunConveyorCommand.h"

RunConveyorCommand::RunConveyorCommand(Conveyor* subsystem, bool forward)
    : m_conveyorSubsystem(subsystem), m_forward(forward) {
  AddRequirements({subsystem});
}

void RunConveyorCommand::Initialize() {
  if (m_forward) {
    m_conveyorSubsystem->Forward();
  } else {
    m_conveyorSubsystem->Reverse();
  }
}

void RunConveyorCommand::End(bool interrupted) {
  m_conveyorSubsystem->Stop();
}

bool RunConveyorCommand::IsFinished() {
  // The command will finish if the limit switch is triggered, handled internally by the subsystem
  return false; // This keeps the command running until explicitly canceled or interrupted
}
