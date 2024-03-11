#include "commands2/base/PivotIntakeCommand.h"  // include command header

PivotIntakeCommand::PivotIntakeCommand(
    Intake* intakeSubsystem, double position,
    bool down)  // add all the relevant subsystems to constructor
    : m_intakeSubsystem(intakeSubsystem), m_position(position), m_down(down) {
  AddRequirements({intakeSubsystem});
}

void PivotIntakeCommand::Initialize() {}

void PivotIntakeCommand::Execute() {
  m_intakeSubsystem->PivotToAngle(m_position, m_down);
}

bool PivotIntakeCommand::IsFinished() { return true; }
