#include "commands/IntakePickUpNoteCommand.h"

IntakePickUpNoteCommand::IntakePickUpNoteCommand(Intake* intakeSubsystem, bool isPressed, double speed)
: m_intakeSubsystem(intakeSubsystem), m_isPressed(isPressed), m_speed(speed) {
    AddRequirements({intakeSubsystem});
}

void IntakePickUpNoteCommand::Initialize() {
    m_intakeSubsystem->IntakePickUpNote(m_isPressed, m_speed);
}
