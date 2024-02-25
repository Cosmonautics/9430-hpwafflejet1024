#ifndef MOVE_TO_TRANSIT_POSITION_COMMAND_H
#define MOVE_TO_TRANSIT_POSITION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>
#include "subsystems/Elevator.h"
#include "subsystems/Shooter.h" 
#include "subsystems/Intake.h" 

class MoveToTransitPositionCommand : public frc2::CommandHelper<frc2::Command, MoveToTransitPositionCommand> {
public:
    MoveToTransitPositionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem, Intake* intakeSubsystem);
    
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Elevator* m_elevatorSubsystem;
    Shooter* m_shooterSubsystem;
    Intake* m_intakeSubsystem;
};

#endif // MOVE_TO_TRANSIT_POSITION_COMMAND_H