#include "commands/MoveToClimbPos1Command.h"  // include command header

MoveToClimbPos1Command::MoveToClimbPos1Command(Elevator* elevatorSubsystem,
                                               Shooter* shooterSubsystem,
                                               Intake* intakeSubsystem)
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_intakeSubsystem(intakeSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem, intakeSubsystem});
}

void MoveToClimbPos1Command::Initialize() {}

void MoveToClimbPos1Command::Execute() {
  m_shooterSubsystem->InvertMotor(true);
  m_shooterSubsystem->PivotToSetPoint(
      PositionConstants::kShooterShooterPosition);
  m_intakeSubsystem->PivotToAngle(PositionConstants::kIntakeClimb1Position,
                                  false);
  // frc2::WaitCommand(0.5_s).Schedule();
  m_elevatorSubsystem->MoveToPosition(
      PositionConstants::kElevatorClimb1Position, false);
}

bool MoveToClimbPos1Command::IsFinished() {
  return m_elevatorSubsystem->AtTargetPosition() &&
         m_shooterSubsystem->IsAtSetPoint() &&
         m_intakeSubsystem->IsAtSetPoint();
}