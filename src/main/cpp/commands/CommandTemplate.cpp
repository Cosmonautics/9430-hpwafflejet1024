#include "commands/CommandTemplate.h" // include command header 

CommandTemplate::CommandTemplate( // Find and Replace "CommandTemplate" with the name of the command 
    Elevator* elevatorSubsystem, Shooter* shooterSubsystem,
    Intake* intakeSubsystem) // add all the relevant subsystems to constructor 
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_intakeSubsystem(intakeSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem, intakeSubsystem});
}

void CommandTemplate::Initialize() {}

void CommandTemplate::Execute() {
  // Put code here 

}

bool CommandTemplate::IsFinished() {
  // Return code here
  // return ...;
}
