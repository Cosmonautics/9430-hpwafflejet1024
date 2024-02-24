#ifndef INTAKE_PICK_UP_NOTE_COMMAND_H
#define INTAKE_PICK_UP_NOTE_COMMAND_H

#include <frc2/command/InstantCommand.h>
#include "subsystems/Intake.h"

class IntakePickUpNoteCommand : public frc2::InstantCommand {
public:
    IntakePickUpNoteCommand(Intake* intakeSubsystem, bool isPressed, double speed);
    
    void Initialize() override;
    
private:
    Intake* m_intakeSubsystem;
    bool m_isPressed;
    double m_speed;
};

#endif // INTAKE_PICK_UP_NOTE_COMMAND_H
