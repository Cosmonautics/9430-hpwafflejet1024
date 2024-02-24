#include "commands/IntakeDropNoteCommand.h"

IntakeDropNoteCommand::IntakeDropNoteCommand(Intake* intakeSubsystem, bool isPressed, double speed)
: m_intakeSubsystem(intakeSubsystem), m_isPressed(isPressed), m_speed(speed) {
    AddRequirements({intakeSubsystem});
}

void IntakeDropNoteCommand::Initialize() {
    m_intakeSubsystem->IntakeDropNote(m_isPressed, m_speed);
}
