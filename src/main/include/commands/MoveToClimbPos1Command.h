#ifndef MOVE_TO_CLIMB_POS_1_COMMAND_H
#define MOVE_TO_CLIMB_POS_1_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>
#include "subsystems/Elevator.h"
#include "subsystems/Shooter.h"  
#include "subsystems/Intake.h"  

class MoveToClimbPos1Command : public frc2::CommandHelper<frc2::Command, MoveToClimbPos1Command> {
public:
    MoveToClimbPos1Command(Elevator* elevatorSubsystem, Shooter* shooterSubsystem, Intake* intakeSubsystem);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    frc::Timer* timer;

private:
    Elevator* m_elevatorSubsystem;
    Shooter* m_shooterSubsystem;
    Intake* m_intakeSubsystem;
};

#endif // MOVE_TO_CLIMB_POS_1_COMMAND_H