#include "commands/ClimbActionCommand.h"

ClimbActionCommand::ClimbActionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem)
: m_elevatorSubsystem(elevatorSubsystem), m_shooterSubsystem(shooterSubsystem) {
    AddRequirements({elevatorSubsystem, shooterSubsystem});
}

void ClimbActionCommand::Initialize() {
    // EL brake piston to lock position
    // Move elevator to climb position
    m_elevatorSubsystem->MoveToPosition(ElevatorConstants::kClimbPositionInches);
    // Pivot shooter/manipulator to trap position
    m_shooterSubsystem->PivotToSetPoint(ShooterConstants::kTrapPositionDegrees);
    // Wait for elevator to get done moving before starting
    frc2::WaitCommand(2_s).Schedule();
    // SM trap piston to score position
    // SM feeder motor out (~50%)
    // SM shooter motor out (~10%)
    // Delay for ~2 seconds
    // SM feeder motor off
    // SM shooter motor off
}

void ClimbActionCommand::Execute() {}

bool ClimbActionCommand::IsFinished() {}