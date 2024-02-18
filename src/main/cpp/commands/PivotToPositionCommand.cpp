#include "commands/PivotToPositionCommand.h"

PivotToPositionCommand::PivotToPositionCommand(Shooter* shooter,
                                               double setPoint)
    : m_shooter(shooter), m_setPoint(setPoint) {
  AddRequirements({shooter});  // Declare subsystem dependencies
}

void PivotToPositionCommand::Initialize() {}
void PivotToPositionCommand::Execute() {
  m_shooter->PivotToSetPoint(m_setPoint);
}
bool PivotToPositionCommand::IsFinished() {
  // Assuming you implement this method in the Shooter subsystem
  return m_shooter->IsAtSetPoint();
}
