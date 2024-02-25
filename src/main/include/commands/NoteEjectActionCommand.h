#ifndef NOTE_EJECT_ACTION_COMMAND_H
#define NOTE_EJECT_ACTION_COMMAND_H

#include <frc2/command/InstantCommand.h>
#include "subsystems/Intake.h"

class NoteEjectActionCommand : public frc2::InstantCommand {
public:
    NoteEjectActionCommand(Intake* intakeSubsystem, bool isPressed, double speed);
    
    void Initialize() override;
    
private:
    Intake* m_intakeSubsystem;
    bool m_isPressed;
    double m_speed;
};

#endif // INTAKE_DROP_NOTE_COMMAND_H
