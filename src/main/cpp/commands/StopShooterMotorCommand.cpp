#include "commands/StopShooterMotorCommand.h"  // include command header

StopShooterMotorCommand::StopShooterMotorCommand(  // Find and Replace
                                                 // "StopShooterMotorCommand"
                                                 // with the name of the command
    Shooter* shooterSubsystem)  // add all the relevant subsystems to constructor
    : m_shooterSubsystem(shooterSubsystem) {
  AddRequirements({shooterSubsystem});
}

void StopShooterMotorCommand::Initialize() {
  cmdFinished = false; // tests should go here to set target states for completion to return true
}

void StopShooterMotorCommand::Execute() { 
  m_shooterSubsystem->StopMotors(); 
  cmdFinished = true; // should only conditionally set to true if and only if tests for motor states return true
}

bool StopShooterMotorCommand::IsFinished() { 
  return cmdFinished; // finished logic should ensure test conditions pass (use motor position/motor state methods)
}
