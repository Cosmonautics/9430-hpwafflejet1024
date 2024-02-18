#include "commands/PivotShooterToTransitPositionCommand.h"

PivotShooterToTransitPositionCommand::PivotShooterToTransitPositionCommand(Shooter* shooterSubsystem)
: m_shooterSubsystem(shooterSubsystem) {
    AddRequirements({shooterSubsystem});
}

void PivotShooterToTransitPositionCommand::Initialize() {
    m_shooterSubsystem->PivotToSetPoint(ShooterConstants::kTransitPositionDegrees);
}

bool PivotShooterToTransitPositionCommand::IsFinished() {
    return m_shooterSubsystem->IsAtSetPoint();
}
