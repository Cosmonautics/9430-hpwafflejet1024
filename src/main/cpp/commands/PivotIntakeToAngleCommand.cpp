#include "commands/PivotIntakeToAngleCommand.h"

PivotIntakeToAngleCommand::PivotIntakeToAngleCommand(Intake* intakeSubsystem, double intakeAngleDegrees)
: m_intakeSubsystem(intakeSubsystem), m_intakeAngleDegrees(intakeAngleDegrees) {
    AddRequirements({intakeSubsystem});
}

void PivotIntakeToAngleCommand::Initialize() {
    m_intakeSubsystem->PivotToAngle(m_intakeAngleDegrees);
}
