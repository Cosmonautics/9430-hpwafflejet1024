#ifndef DO_NOTE_EJECT_ACTION_COMMAND_H
#define DO_NOTE_EJECT_ACTION_COMMAND_H 

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>
#include "subsystems/Conveyor.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h" 

class DoNoteEjectActionCommand : public frc2::CommandHelper<frc2::Command, DoNoteEjectActionCommand> {
public:
    DoNoteEjectActionCommand(Conveyor* conveyorSubsystem, Shooter* shooterSubsystem, Intake* intakeSubsystem); // Put in all relevant subsystems

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Conveyor* m_conveyorSubsystem; // Put all relevant subsystems here as pointers
    Shooter* m_shooterSubsystem;
    Intake* m_intakeSubsystem;
};

#endif // DO_NOTE_EJECT_ACTION_COMMAND_H