#include "commands/NoteIntakeActionCommand.h"

NoteIntakeActionCommand::NoteIntakeActionCommand(Intake* intakeSubsystem, bool isPressed, double speed)
: m_intakeSubsystem(intakeSubsystem), m_isPressed(isPressed), m_speed(speed) {
    AddRequirements({intakeSubsystem});
}

void NoteIntakeActionCommand::Initialize() {
    m_intakeSubsystem->IntakePickUpNote(m_isPressed, m_speed);
}
