#include "commands/PivotShooterToPositionCommand.h"

PivotShooterToPositionCommand::PivotShooterToPositionCommand(Shooter* shooter,
                                               double setPoint)
    : m_shooter(shooter), m_setPoint(setPoint) {
  AddRequirements({shooter});  // Declare subsystem dependencies
}

void PivotShooterToPositionCommand::Initialize() {
  m_shooter->PivotToSetPoint(m_setPoint);
}

void PivotShooterToPositionCommand::Execute() {}

bool PivotShooterToPositionCommand::IsFinished() {
  // Assuming you implement this method in the Shooter subsystem
  return m_shooter->IsAtSetPoint();
}
