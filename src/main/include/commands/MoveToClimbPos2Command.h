#ifndef MOVE_TO_CLIMB_POS_2_COMMAND_H
#define MOVE_TO_CLIMB_POS_2_COMMAND_H 

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h" 

class MoveToClimbPos2Command : public frc2::CommandHelper<frc2::Command, MoveToClimbPos2Command> {
public:
    MoveToClimbPos2Command(Elevator* elevatorSubsystem);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Elevator* m_elevatorSubsystem; 
};

#endif // MOVE_TO_CLIMB_POS_2_COMMAND_H 