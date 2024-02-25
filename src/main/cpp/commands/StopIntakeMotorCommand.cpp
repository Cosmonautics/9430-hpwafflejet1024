#include "commands/StopIntakeMotorCommand.h"  // include command header

StopIntakeMotorCommand::StopIntakeMotorCommand(  // Find and Replace
                                                 // "StopIntakeMotorCommand"
                                                 // with the name of the command
    Intake* intakeSubsystem)  // add all the relevant subsystems to constructor
    : m_intakeSubsystem(intakeSubsystem) {
  AddRequirements({intakeSubsystem});
}

void StopIntakeMotorCommand::Initialize() {}

void StopIntakeMotorCommand::Execute() { m_intakeSubsystem->StopMotors(); }

bool StopIntakeMotorCommand::IsFinished() { return true; }
