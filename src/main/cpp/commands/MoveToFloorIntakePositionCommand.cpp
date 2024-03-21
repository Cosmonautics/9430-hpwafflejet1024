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
  m_shooterSubsystem->InvertMotor(true);
  m_shooterSubsystem->StopMotors();
  m_shooterSubsystem->MoveFeeder(0.0);
  m_shooterSubsystem->PivotToSetPoint(
      PositionConstants::kShooterTransitPosition);
  m_intakeSubsystem->PivotToAngle(PositionConstants::kIntakeFloorPosition,
                                  true);
  // frc2::WaitCommand(0.5_s).Execute();
  m_elevatorSubsystem->MoveToPosition(
      PositionConstants::kElevatorTransitPosition, false);
}

bool MoveToFloorIntakePositionCommand::IsFinished() {
  return true;
}
