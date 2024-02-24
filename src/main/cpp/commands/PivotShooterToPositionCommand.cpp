#include "commands/PivotShooterToPositionCommand.h"

PivotShooterToPositionCommand::PivotShooterToPositionCommand(Shooter* shooter,
                                                             double setPoint)
    : m_shooter(shooter), m_setPoint(setPoint) {
  AddRequirements({shooter});  // Declare subsystem dependencies
}

void PivotShooterToPositionCommand::Initialize() {
}

void PivotShooterToPositionCommand::Execute() {
   m_shooter->PivotToSetPoint(m_setPoint);
}

bool PivotShooterToPositionCommand::IsFinished() {
  return m_shooter->IsAtSetPoint();
}
