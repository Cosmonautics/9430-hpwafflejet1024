#include "commands2/base/MoveShooterWheelsCommand.h"  // include command header

MoveShooterWheelsCommand::MoveShooterWheelsCommand(
    Shooter* shooterSubsystem,
    double speed)  // add all the relevant subsystems to constructor
    : m_shooterSubsystem(shooterSubsystem), m_speed(speed) {
  AddRequirements({shooterSubsystem});
}

void MoveShooterWheelsCommand::Initialize() {}

void MoveShooterWheelsCommand::Execute() {
  m_shooterSubsystem->ShootMotors(true, m_speed);
}

bool MoveShooterWheelsCommand::IsFinished() { return true; }
