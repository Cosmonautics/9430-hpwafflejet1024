#include "commands/DoSourceIntakeActionCommand.h"  // include command header

DoSourceIntakeActionCommand::
    DoSourceIntakeActionCommand(  // Find and Replace
                                  // "DoSourceIntakeActionCommand"
                                  // with the name of
                                  // the command
        Elevator* elevatorSubsystem,
        Shooter*
            shooterSubsystem)  // add all the relevant subsystems to constructor
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem});
}

void DoSourceIntakeActionCommand::Initialize() {}

void DoSourceIntakeActionCommand::Execute() {
  m_shooterSubsystem->InvertMotor(true);
  m_shooterSubsystem->PivotToSetPoint(PositionConstants::kShooterIntakeSourcePosition);
  // frc2::WaitCommand(0.8_s).Schedule();
  m_elevatorSubsystem->MoveToPosition(
      PositionConstants::kElevatorSourceIntakePosition, false);
  m_shooterSubsystem->ShooterPickUpNote(true, 0.30);
  m_shooterSubsystem->MoveFeeder(1);
}

bool DoSourceIntakeActionCommand::IsFinished() { return true; }
