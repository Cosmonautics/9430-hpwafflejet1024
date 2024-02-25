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
  cmdFinished = false;
}

void DoNoteEjectActionCommand::Execute() {
  m_intakeSubsystem->IntakeDropNote(true, 1);
  m_conveyorSubsystem->Reverse();
  m_shooterSubsystem->ShooterDropNote(true, 0.10);
  m_shooterSubsystem->MoveFeeder(-1);
  cmdFinished = true;
}

bool DoNoteEjectActionCommand::IsFinished() {
  return cmdFinished;
}
