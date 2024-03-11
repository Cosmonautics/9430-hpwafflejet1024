#include "commands2/base/MoveShooterFeederWheelsCommand.h"  // include command header

MoveShooterFeederWheelsCommand::MoveShooterFeederWheelsCommand(
    Shooter* shooterSubsystem,
    double speed)  // add all the relevant subsystems to constructor
    : m_shooterSubsystem(shooterSubsystem), m_speed(speed) {
  AddRequirements({shooterSubsystem});
}

void MoveShooterFeederWheelsCommand::Initialize() {}

void MoveShooterFeederWheelsCommand::Execute() {
  m_shooterSubsystem->MoveFeeder(m_speed);
}

bool MoveShooterFeederWheelsCommand::IsFinished() { return true; }
