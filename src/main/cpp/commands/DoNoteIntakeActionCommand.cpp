#include "commands/DoNoteIntakeActionCommand.h"  // include command header

DoNoteIntakeActionCommand::DoNoteIntakeActionCommand(  // Find and Replace
                                                       // "CommandTemplate" with
                                                       // the name of the
                                                       // command
    Conveyor* conveyorSubsystem, Shooter* shooterSubsystem,
    Intake* intakeSubsystem)  // add all the relevant subsystems to constructor
    : m_conveyorSubsystem(conveyorSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_intakeSubsystem(intakeSubsystem) {
  AddRequirements({conveyorSubsystem, shooterSubsystem, intakeSubsystem});
}

void DoNoteIntakeActionCommand::Initialize() {
  cmdFinished = false;
}

void DoNoteIntakeActionCommand::Execute() {
  m_intakeSubsystem->IntakePickUpNote(true, 1);
  m_conveyorSubsystem->Forward();
  m_shooterSubsystem->ShooterPickUpNote(true, 0.30);
  m_shooterSubsystem->MoveFeeder(1);
  cmdFinished = true;
}

bool DoNoteIntakeActionCommand::IsFinished() { 
  return cmdFinished; 
}
