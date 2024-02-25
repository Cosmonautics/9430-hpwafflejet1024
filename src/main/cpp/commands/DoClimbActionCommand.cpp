#include "commands/DoClimbActionCommand.h" // include command header 

DoClimbActionCommand::DoClimbActionCommand(
    Elevator* elevatorSubsystem, Shooter* shooterSubsystem
    ): 
      m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem});
}

void DoClimbActionCommand::Initialize() {}

void DoClimbActionCommand::Execute() {
  // Put code here 

}

bool DoClimbActionCommand::IsFinished() {
  // Return code here
  // return ...;
}
