#include "commands/MoveToAmpSpeakerScorePositionCommand.h"

MoveToAmpSpeakerScorePositionCommand::MoveToAmpSpeakerScorePositionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem)
: m_elevatorSubsystem(elevatorSubsystem), m_shooterSubsystem(shooterSubsystem) {
    AddRequirements({elevatorSubsystem, shooterSubsystem});
    }

void MoveToAmpSpeakerScorePositionCommand::Initialize() {
    m_elevatorSubsystem->MoveToPosition(ElevatorConstants::kTransitPositionRotations);
    m_shooterSubsystem->PivotToSetPoint(0.88);
}

void MoveToAmpSpeakerScorePositionCommand::Execute() {}

bool MoveToAmpSpeakerScorePositionCommand::IsFinished() {
    return m_elevatorSubsystem->AtTargetPosition();
    return m_shooterSubsystem->IsAtSetPoint();
}