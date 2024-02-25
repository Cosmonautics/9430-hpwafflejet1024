#include "commands/MoveToFloorIntakePositionCommand.h"

MoveToFloorIntakePositionCommand::MoveToFloorIntakePositionCommand(
    Elevator* elevatorSubsystem, Shooter* shooterSubsystem,
    Intake* intakeSubsystem)
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_intakeSubsystem(intakeSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem, intakeSubsystem});
}

void MoveToFloorIntakePositionCommand::Initialize() {}

void MoveToFloorIntakePositionCommand::Execute() {
  m_elevatorSubsystem->MoveToPosition(
      ElevatorConstants::kFloorIntakePositionRotations);
  m_shooterSubsystem->PivotToSetPoint(
      ShooterConstants::kFloorIntakePositionRotations);
  m_intakeSubsystem->PivotToAngle(IntakeConstants::kFloorIntakeAngle);
}

bool MoveToFloorIntakePositionCommand::IsFinished() {
  return m_elevatorSubsystem->AtTargetPosition();
}
