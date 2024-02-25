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
  // Execute when Y is pressed; if Y is pressed again, stop the climb sequence 
  // Set EL brake piston to lock position 
  // Move elevator to climb position (SM carriage to the top)
  // Move shooter to trap position 
    // Wait for elevator to get done moving
    // Set shooter to AMP scoring mode (TBD may be tweaked)
      // basically, repeat the same code in AMP scoring mode.
  // Set shooter trap piston to score position
  // Set shooter feeder motor to shoot out at ~50%
    // Wait for ~2 seconds
  // Set shooter feeder motor 0%
  // Set shooter motor 0%

}

bool DoClimbActionCommand::IsFinished() {
  // Return code here
  // return ...;
}
