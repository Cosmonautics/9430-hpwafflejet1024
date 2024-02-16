#include "commands/GoToFloorIntakePositionCommand.h"

GoToFloorIntakePositionCommand::GoToFloorIntakePositionCommand(Elevator& elevator,Shooter& shooter,Intake& intake)
    : m_elevator(elevator), m_shooter(shooter), m_intake(intake){}


void GoToFloorIntakePositionCommand::Execute() {
    m_elevator.MoveToPosition(CommandConstants::kFLoorIntakePostionElevator);
    m_shooter.PivotToSetPoint(CommandConstants::kFloorIntakePositionShooter);
    m_intake.PivotToAngle(CommandConstants::kFLoorIntakePostionIntake);
}
