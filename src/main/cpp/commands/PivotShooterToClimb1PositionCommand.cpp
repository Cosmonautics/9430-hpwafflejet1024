#include "commands/PivotShooterToClimb1PositionCommand.h"

PivotShooterToClimb1PositionCommand::PivotShooterToClimb1PositionCommand(Shooter* shooterSubsystem)
: m_shooterSubsystem(shooterSubsystem) {
    AddRequirements({shooterSubsystem});
}

void PivotShooterToClimb1PositionCommand::Initialize() {
    m_shooterSubsystem->PivotToSetPoint(ShooterConstants::kClimb1PositionDegrees);
}

bool PivotShooterToClimb1PositionCommand::IsFinished() {
    return m_shooterSubsystem->IsAtSetPoint();
}
