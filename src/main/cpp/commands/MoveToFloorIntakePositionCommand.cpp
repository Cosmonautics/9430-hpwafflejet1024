#include "commands/MoveToFloorIntakePositionCommand.h"

MoveToFloorIntakePositionCommand::MoveToFloorIntakePositionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem, Intake* intakeSubsystem)
: m_elevatorSubsystem(elevatorSubsystem), m_shooterSubsystem(shooterSubsystem), m_intakeSubsystem(intakeSubsystem) {
    AddRequirements({elevatorSubsystem, shooterSubsystem, intakeSubsystem});
}


void MoveToFloorIntakePositionCommand::Initialize() {
    m_elevatorSubsystem->MoveToPosition(ElevatorConstants::kFloorIntakePositionInches);
    m_shooterSubsystem->PivotToSetPoint(ShooterConstants::kFloorIntakePositionDegrees);
}

void MoveToFloorIntakePositionCommand::Execute() {}

bool MoveToFloorIntakePositionCommand::IsFinished() {
    return m_elevatorSubsystem->AtTargetPosition();
    return m_shooterSubsystem->IsAtSetPoint();
}
