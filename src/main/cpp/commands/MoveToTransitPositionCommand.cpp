#include "commands/MoveToTransitPositionCommand.h"

MoveToTransitPositionCommand::MoveToTransitPositionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem, Intake* intakeSubsystem)
: m_elevatorSubsystem(elevatorSubsystem), m_shooterSubsystem(shooterSubsystem), m_intakeSubsystem(intakeSubsystem) {
    AddRequirements({elevatorSubsystem, shooterSubsystem, intakeSubsystem});
}

void MoveToTransitPositionCommand::Initialize() {
    m_elevatorSubsystem->MoveToPosition(ElevatorConstants::kTransitPositionRotations);
    m_shooterSubsystem->PivotToSetPoint(ShooterConstants::kTransitPositionDegrees);
}

void MoveToTransitPositionCommand::Execute() {
    
}

bool MoveToTransitPositionCommand::IsFinished() {
    return m_elevatorSubsystem->AtTargetPosition();
    return m_shooterSubsystem->IsAtSetPoint();
}
