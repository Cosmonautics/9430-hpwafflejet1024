#ifndef STOP_NOTE_INTAKE_COMMAND_H
#define STOP_NOTE_INTAKE_COMMAND_H // rename these to the name of the command in all caps followed by _H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>
#include "subsystems/Conveyor.h" // import all relevant subsystems 
#include "subsystems/Shooter.h" // import all relevant subsystems 
#include "subsystems/Intake.h" // import all relevant subsystems 

class StopNoteIntakeActionCommand : public frc2::CommandHelper<frc2::Command, StopNoteIntakeActionCommand> {
public:
    StopNoteIntakeActionCommand(Conveyor* conveyorSubsystem, Shooter* shooterSubsystem, Intake* intakeSubsystem); // Put in all relevant subsystems

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Conveyor* m_conveyorSubsystem; // Put all relevant subsystems here as pointers
    Shooter* m_shooterSubsystem;
    Intake* m_intakeSubsystem;
};

#endif // DO_NOTE_INTAKE_COMMAND_H // (replace this with the name of the command name)