#include "commands/DoSpeakerScoreActionCommand.h" 

DoSpeakerScoreActionCommand::DoSpeakerScoreActionCommand( 
    Elevator* elevatorSubsystem, Shooter* shooterSubsystem) // add all the relevant subsystems to constructor 
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem});
}

void DoSpeakerScoreActionCommand::Initialize() {}

void DoSpeakerScoreActionCommand::Execute() {
  // Put code here 

}

bool DoSpeakerScoreActionCommand::IsFinished() {
  // Return code here
  // return ...;
}
