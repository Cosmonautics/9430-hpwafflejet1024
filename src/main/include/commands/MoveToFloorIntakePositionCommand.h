#ifndef MOVE_TO_FLOOR_INTAKE_POSITION_COMMAND_H
#define MOVE_TO_FLOOR_INTAKE_POSITION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

class MoveToFloorIntakePositionCommand : public frc2::CommandHelper<frc2::Command, MoveToFloorIntakePositionCommand> {
public:
    MoveToFloorIntakePositionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem, Intake* intakeSubsystem);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Elevator* m_elevatorSubsystem;
    Shooter* m_shooterSubsystem;
    Intake* m_intakeSubsystem;
};

#endif // MOVE_ELEVATOR_TO_FLOOR_INTAKE_POSITION_COMMAND_H