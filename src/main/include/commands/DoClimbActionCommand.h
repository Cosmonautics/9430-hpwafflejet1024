#ifndef DO_CLIMB_ACTION_COMMAND_H
#define DO_CLIMB_ACTION_COMMAND_H 

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>
#include "subsystems/Elevator.h" 
#include "subsystems/Shooter.h" 

class DoClimbActionCommand : public frc2::CommandHelper<frc2::Command, DoClimbActionCommand> {
public:
    DoClimbActionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem, bool isClimb1); 

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Elevator* m_elevatorSubsystem; 
    Shooter* m_shooterSubsystem;
    bool m_isClimb1;
    bool cmdFinished;
};

#endif // DO_CLIMB_ACTION_COMMAND_H