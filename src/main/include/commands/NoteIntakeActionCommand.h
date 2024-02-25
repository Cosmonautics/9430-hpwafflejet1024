#ifndef NOTE_INTAKE_ACTION_COMMAND_H
#define NOTE_INTAKE_ACTION_COMMAND_H

#include <frc2/command/InstantCommand.h>
#include "subsystems/Intake.h"

class NoteIntakeActionCommand : public frc2::InstantCommand {
public:
    NoteIntakeActionCommand(Intake* intakeSubsystem, bool isPressed, double speed);
    
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override; 
    
private:
    Intake* m_intakeSubsystem;
    bool m_isPressed;
    double m_speed;
};

#endif // INTAKE_PICK_UP_NOTE_COMMAND_H
