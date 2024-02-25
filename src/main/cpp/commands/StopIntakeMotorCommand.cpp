#include "commands/StopIntakeMotorCommand.h"  // include command header

StopIntakeMotorCommand::StopIntakeMotorCommand(  // Find and Replace
                                                 // "StopIntakeMotorCommand"
                                                 // with the name of the command
    Intake* intakeSubsystem)  // add all the relevant subsystems to constructor
    : m_intakeSubsystem(intakeSubsystem) {
  AddRequirements({intakeSubsystem});
}

void StopIntakeMotorCommand::Initialize() {
  cmdFinished = false; // tests should go here to set target states for completion to return true
}

void StopIntakeMotorCommand::Execute() { 
  m_intakeSubsystem->StopMotors(); 
  cmdFinished = true; // should only conditionally set to true if and only if tests for motor states return true
}

bool StopIntakeMotorCommand::IsFinished() { 
  return cmdFinished; // finished logic should ensure test conditions pass (use motor position/motor state methods)
}
