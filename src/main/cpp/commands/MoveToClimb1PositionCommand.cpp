#include "commands/MoveToClimb1PositionCommand.h"

MoveToClimb1PositionCommand::MoveToClimb1PositionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem, Intake* intakeSubsystem)
: m_elevatorSubsystem(elevatorSubsystem) {
    AddRequirements({elevatorSubsystem, shooterSubsystem, intakeSubsystem});
}

void MoveToClimb1PositionCommand::Initialize() {
    m_elevatorSubsystem->MoveToPosition(ElevatorConstants::kClimb1PositionInches);
    m_shooterSubsystem->PivotToSetPoint(ShooterConstants::kClimb1PositionDegrees);
}

void MoveToClimb1PositionCommand::Execute() {}

bool MoveToClimb1PositionCommand::IsFinished() {
    return m_elevatorSubsystem->AtTargetPosition();
    return m_shooterSubsystem->IsAtSetPoint();
}

