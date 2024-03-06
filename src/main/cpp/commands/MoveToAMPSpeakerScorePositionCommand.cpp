#include "commands/MoveToAMPSpeakerScorePositionCommand.h"

MoveToAMPSpeakerScorePositionCommand::MoveToAMPSpeakerScorePositionCommand(
    Elevator* elevatorSubsystem, Shooter* shooterSubsystem)
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem});
}

void MoveToAMPSpeakerScorePositionCommand::Initialize() {}

void MoveToAMPSpeakerScorePositionCommand::Execute() {
  m_shooterSubsystem->InvertMotor(true);
  m_shooterSubsystem->PivotToSetPoint(
      PositionConstants::kShooterPreShooterPosition);
  // frc2::WaitCommand(0.8_s).Schedule();
  m_elevatorSubsystem->MoveToPosition(
      PositionConstants::kElevatorShooterPosition, false);
}

bool MoveToAMPSpeakerScorePositionCommand::IsFinished() {
  return m_elevatorSubsystem->AtTargetPosition() &&
         m_shooterSubsystem->IsAtSetPoint();
}
