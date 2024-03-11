#include "commands2/base/PivotShooterCommand.h"  // include command header

PivotShooterCommand::PivotShooterCommand(
    Shooter* shooterSubsystem,
    double position)  // add all the relevant subsystems to constructor
    : m_shooterSubsystem(shooterSubsystem), m_position(position) {
  AddRequirements({shooterSubsystem});
}

void PivotShooterCommand::Initialize() {}

void PivotShooterCommand::Execute() {
  m_shooterSubsystem->PivotToSetPoint(m_position);
}

bool PivotShooterCommand::IsFinished() { return true; }
