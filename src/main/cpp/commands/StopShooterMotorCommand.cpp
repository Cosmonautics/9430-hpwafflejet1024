#include "commands/StopShooterMotorCommand.h"  // include command header

StopShooterMotorCommand::StopShooterMotorCommand(  // Find and Replace
                                                 // "StopShooterMotorCommand"
                                                 // with the name of the command
    Shooter* shooterSubsystem)  // add all the relevant subsystems to constructor
    : m_shooterSubsystem(shooterSubsystem) {
  AddRequirements({shooterSubsystem});
}

void StopShooterMotorCommand::Initialize() {}

void StopShooterMotorCommand::Execute() { m_shooterSubsystem->StopMotors(); }

bool StopShooterMotorCommand::IsFinished() { return true; }
