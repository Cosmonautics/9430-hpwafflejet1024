#include "commands/DoNoteEjectActionCommand.h"

DoNoteEjectActionCommand::DoNoteEjectActionCommand(
    Conveyor* conveyorSubsystem, Shooter* shooterSubsystem,
    Intake* intakeSubsystem)  // add all the relevant subsystems to constructor
    : m_conveyorSubsystem(conveyorSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_intakeSubsystem(intakeSubsystem) {
  AddRequirements({conveyorSubsystem, shooterSubsystem, intakeSubsystem});
}

void DoNoteEjectActionCommand::Initialize() {
  cmdFinished = false; // tests should go here to set target states for completion to return true
}

void DoNoteEjectActionCommand::Execute() {
  m_intakeSubsystem->IntakeDropNote(true, 1);
  m_conveyorSubsystem->Reverse();
  m_shooterSubsystem->ShooterDropNote(true, 0.10);
  m_shooterSubsystem->MoveFeeder(-1);
  cmdFinished = true; // should only conditionally set to true if and only if tests for motor states return true
}

bool DoNoteEjectActionCommand::IsFinished() {
  return cmdFinished; // finished logic should ensure test conditions pass (use motor position/motor state methods)
}
