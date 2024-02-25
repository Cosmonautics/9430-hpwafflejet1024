#include "commands/MoveToTransitPositionCommand.h"  // include command header

MoveToTransitPositionCommand::MoveToTransitPositionCommand(
    Elevator* elevatorSubsystem, Shooter* shooterSubsystem,
    Intake* intakeSubsystem)
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_intakeSubsystem(intakeSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem, intakeSubsystem});
}

void MoveToTransitPositionCommand::Initialize() {}

void MoveToTransitPositionCommand::Execute() {
  m_shooterSubsystem->PivotToSetPoint(
      PositionConstants::kShooterTransitPosition);
  m_intakeSubsystem->PivotToAngle(PositionConstants::kIntakeTransitPosition);

  // frc2::WaitCommand(0.5_s).Schedule();
  m_elevatorSubsystem->MoveToPosition(
      PositionConstants::kElevatorTransitPosition);
}

bool MoveToTransitPositionCommand::IsFinished() { return true; }
