#include "commands/PivotToPositionCommand.h"

PivotToPositionCommand::PivotToPositionCommand(Shooter* shooter, units::degree_t setPoint)
    : m_shooter(shooter), m_setPoint(setPoint) {
  AddRequirements({shooter}); // Declare subsystem dependencies
}

void PivotToPositionCommand::Initialize() {
  m_shooter->PivotToSetPoint(m_setPoint);
}

bool PivotToPositionCommand::IsFinished() {
  // Assuming you implement this method in the Shooter subsystem
  return m_shooter->IsAtSetPoint();
}
