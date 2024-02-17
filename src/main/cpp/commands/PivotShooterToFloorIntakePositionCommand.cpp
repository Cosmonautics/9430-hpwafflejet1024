#include "commands/PivotShooterToFloorIntakePositionCommand.h"

PivotShooterToFloorIntakePositionCommand::PivotShooterToFloorIntakePositionCommand(Shooter* shooterSubsystem)
: m_shooterSubsystem(shooterSubsystem) {
    AddRequirements({shooterSubsystem});
}

void PivotShooterToFloorIntakePositionCommand::Initialize() {
    m_shooterSubsystem->PivotToSetPoint(ShooterConstants::kFloorIntakePositionDegrees);
}

bool PivotShooterToFloorIntakePositionCommand::IsFinished() {
    return m_shooterSubsystem->IsAtSetPoint();
}
