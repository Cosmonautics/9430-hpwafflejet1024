#include "commands/StopNoteIntakeActionCommand.h"  // include command header

StopNoteIntakeActionCommand::StopNoteIntakeActionCommand(  // Find and Replace
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

void StopNoteIntakeActionCommand::Initialize() {
  cmdFinished = false; // tests should go here to set target states for completion to return true
}

void StopNoteIntakeActionCommand::Execute() {
  m_intakeSubsystem->StopMotors();
  m_conveyorSubsystem->Stop();
  m_shooterSubsystem->StopMotors();
  m_shooterSubsystem->MoveFeeder(0);
  cmdFinished = true; // should only conditionally set to true if and only if tests for motor states return true
}

bool StopNoteIntakeActionCommand::IsFinished() { 
  return cmdFinished; // finished logic should ensure test conditions pass (use motor position/motor state methods)
}
