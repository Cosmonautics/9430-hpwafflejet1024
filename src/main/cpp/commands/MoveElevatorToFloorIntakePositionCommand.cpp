#include "commands/MoveToFloorIntakePositionCommand.h"

MoveToFloorIntakePositionCommand::MoveToFloorIntakePositionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem, Intake intakeSubsystem)
: m_elevatorSubsystem(elevatorSubsystem), m_shooterSubsystem(shooterSubsystem), m_intakeSubsystem(intakeSubsystem) {
    AddRequirements({elevatorSubsystem, shooterSubsystem, intakeSubsystem});
}

void MoveToFloorIntakePositionCommand::Initialize() {
    m_elevatorSubsystem->MoveToPosition(ElevatorConstants::kFloorIntakePositionInches);
}

bool MoveToFloorIntakePositionCommand::IsFinished() {
    return m_elevatorSubsystem->AtTargetPosition();
}



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
