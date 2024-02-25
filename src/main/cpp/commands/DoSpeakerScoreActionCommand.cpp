#include "commands/DoSpeakerScoreActionCommand.h" 

DoSpeakerScoreActionCommand::DoSpeakerScoreActionCommand( 
    Elevator* elevatorSubsystem, Shooter* shooterSubsystem) // add all the relevant subsystems to constructor 
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem});
}

void DoSpeakerScoreActionCommand::Initialize() {}

void DoSpeakerScoreActionCommand::Execute() {
  // Check if AMP Score Position status == TRUE
    // Set shooter motor 100%
    // (TBD VISION) auto align DT to point at the speaker
    // (TBD VISION/PATHING) pivot shooter manipulator to proper angle based on distance from goal
    // Set shooter feeder motor 100% 
      // Wait ~1 second 
    // Set shooter motor 0%
    // Set shooter feeder motor 0% 

}

bool DoSpeakerScoreActionCommand::IsFinished() {
  // Return code here
  // return ...;
}
