#include "commands/ControlIntakeMotorsCommand.h"

ControlIntakeMotorsCommand::ControlIntakeMotorsCommand(Intake* intakeSubsystem, bool isPressed, double speed)
: m_intakeSubsystem(intakeSubsystem), m_isPressed(isPressed), m_speed(speed) {
    AddRequirements({intakeSubsystem});
}

void ControlIntakeMotorsCommand::Initialize() {
    m_intakeSubsystem->ControlIntakeMotors(m_isPressed, m_speed);
}
