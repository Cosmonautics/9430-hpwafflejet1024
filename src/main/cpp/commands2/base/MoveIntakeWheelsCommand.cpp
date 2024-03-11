#include "commands2/base/MoveIntakeWheelsCommand.h"  // include command header

MoveIntakeWheelsCommand::MoveIntakeWheelsCommand(
    Intake* intakeSubsystem, double speed,
    bool pressed)  // add all the relevant subsystems to constructor
    : m_intakeSubsystem(intakeSubsystem), m_speed(speed), m_pressed(pressed) {
  AddRequirements({intakeSubsystem});
}

void MoveIntakeWheelsCommand::Initialize() {}

void MoveIntakeWheelsCommand::Execute() {
  m_intakeSubsystem->IntakePickUpNote(m_pressed, m_speed);
}

bool MoveIntakeWheelsCommand::IsFinished() { return true; }
