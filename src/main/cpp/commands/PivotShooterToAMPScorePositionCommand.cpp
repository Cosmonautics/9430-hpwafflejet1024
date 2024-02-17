#include "commands/PivotShooterToAMPScorePositionCommand.h"

PivotShooterToAMPScorePositionCommand::PivotShooterToAMPScorePositionCommand(Shooter* shooterSubsystem)
: m_shooterSubsystem(shooterSubsystem) {
    AddRequirements({shooterSubsystem});
}

void PivotShooterToAMPScorePositionCommand::Initialize() {
    m_shooterSubsystem->PivotToSetPoint(ShooterConstants::kAMPScorePositionDegrees);
}

bool PivotShooterToAMPScorePositionCommand::IsFinished() {
    return m_shooterSubsystem->IsAtSetPoint();
}
