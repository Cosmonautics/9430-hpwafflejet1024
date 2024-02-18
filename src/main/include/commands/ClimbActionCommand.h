#ifndef CLIMB_ACTION_COMMAND_H
#define CLIMB_ACTION_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Elevator.h"
#include "subsystems/Shooter.h"
#include "frc2/command/WaitCommand.h"

class ClimbActionCommand : public frc2::CommandHelper<frc2::Command, ClimbActionCommand> {
public:
    ClimbActionCommand(Elevator* elevatorSubsystem, Shooter* shooterSubsystem);

    void Initialize() override;

private:
    Elevator* m_elevatorSubsystem;
    Shooter* m_shooterSubsystem;
};

#endif // CLIMB_ACTION_COMMAND_H
