#include "commands/DoNoteIntakeActionCommand.h"  // include command header

DoNoteIntakeActionCommand::DoNoteIntakeActionCommand(  // Find and Replace
                                                       // "CommandTemplate" with
                                                       // the name of the
                                                       // command
    Conveyor* conveyorSubsystem, Shooter* shooterSubsystem,
    Intake* intakeSubsystem,
    Elevator*
        elevatorSubsystem)  // add all the relevant subsystems to constructor
    : m_conveyorSubsystem(conveyorSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_intakeSubsystem(intakeSubsystem),
      m_elevatorSubsystem(elevatorSubsystem) {
  AddRequirements({conveyorSubsystem, shooterSubsystem, intakeSubsystem,
                   elevatorSubsystem});
}

void DoNoteIntakeActionCommand::Initialize() {
  cmdFinished = false;  // tests should go here to set target states for
                        // completion to return true
}

void DoNoteIntakeActionCommand::Execute() {
  m_shooterSubsystem->InvertMotor(true);
  // ENTIRELY EXPERIMENTAL
  m_shooterSubsystem->PivotToSetPoint(
      PositionConstants::kShooterTransitPosition);
  m_elevatorSubsystem->MoveToPosition(
      PositionConstants::kElevatorTransitPosition, false);
  // END --
  m_intakeSubsystem->IntakePickUpNote(true, 1);
  m_conveyorSubsystem->Forward();
  m_shooterSubsystem->ShooterPickUpNote(true, 0.30);
  m_shooterSubsystem->MoveFeeder(1);
  cmdFinished = true;  // should only conditionally set to true if and only if
                       // tests for motor states return true
}

bool DoNoteIntakeActionCommand::IsFinished() {
  return cmdFinished;  // finished logic should ensure test conditions pass (use
                       // motor position/motor state methods)
}
