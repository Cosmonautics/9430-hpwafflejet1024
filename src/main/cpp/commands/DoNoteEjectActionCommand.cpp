#include "commands/DoNoteEjectActionCommand.h" 

DoNoteEjectActionCommand::DoNoteEjectActionCommand( 
    Conveyor* conveyorSubsystem, Shooter* shooterSubsystem,
    Intake* intakeSubsystem) // add all the relevant subsystems to constructor 
    : m_conveyorSubsystem(conveyorSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_intakeSubsystem(intakeSubsystem) {
  AddRequirements({conveyorSubsystem, shooterSubsystem, intakeSubsystem});
}

void DoNoteEjectActionCommand::Initialize() {}

void DoNoteEjectActionCommand::Execute() {
  // Set Intake wheel motor to 100%
  // Set Conveyor motor to 100%
  // Set shooter motor to ~10%
  // Set shooter feeder motor to 100% 

}

bool DoNoteEjectActionCommand::IsFinished() {
  // Return code here
  // return ...;
}
