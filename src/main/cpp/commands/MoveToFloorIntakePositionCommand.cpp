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
  m_shooterSubsystem->PivotToSetPoint(
      PositionConstants::kShooterTransitPosition);
  frc2::WaitCommand(1_s).Schedule();
  m_elevatorSubsystem->MoveToPosition(
      PositionConstants::kElevatorTransitPosition);
  m_intakeSubsystem->PivotToAngle(PositionConstants::kIntakeFloorPosition);
}

bool MoveToFloorIntakePositionCommand::IsFinished() {
  return m_elevatorSubsystem->AtTargetPosition() &&
         m_shooterSubsystem->IsAtSetPoint() &&
         m_intakeSubsystem->IsAtSetPoint();
}
