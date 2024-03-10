#include "commands/MoveToTransitPositionCommand.h"  // include command header

MoveToTransitPositionCommand::MoveToTransitPositionCommand(
    Elevator* elevatorSubsystem, Shooter* shooterSubsystem,
    Intake* intakeSubsystem)
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_intakeSubsystem(intakeSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem, intakeSubsystem});
}

void MoveToTransitPositionCommand::Initialize() {
  cmdFinished = true;  // tests should go here to set target states for
                       // completion to return true
}

void MoveToTransitPositionCommand::Execute() {
  m_shooterSubsystem->InvertMotor(true);
  m_shooterSubsystem->StopMotors();
  m_shooterSubsystem->MoveFeeder(0.0);
  m_shooterSubsystem->PivotToSetPoint(
      PositionConstants::kShooterTransitPosition);
  m_intakeSubsystem->PivotToAngle(PositionConstants::kIntakeTransitPosition,
                                  false);

  // frc2::WaitCommand(0.5_s).Schedule();
  m_elevatorSubsystem->MoveToPosition(
      PositionConstants::kElevatorTransitPosition, false);
  cmdFinished = true;  // should only conditionally set to true if and only if
                       // tests for motor states return true
}

bool MoveToTransitPositionCommand::IsFinished() {
  return cmdFinished;  // finished logic should ensure test conditions pass (use
                       // motor position/motor state methods)
}
