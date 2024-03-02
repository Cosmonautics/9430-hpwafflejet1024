#include "commands/StopSourceIntakeActionCommand.h"  // include command header

StopSourceIntakeActionCommand::
    StopSourceIntakeActionCommand(  // Find and Replace
                                  // "StopSourceIntakeActionCommand"
                                  // with the name of
                                  // the command
        Elevator* elevatorSubsystem,
        Shooter*
            shooterSubsystem)  // add all the relevant subsystems to constructor
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem});
}

void StopSourceIntakeActionCommand::Initialize() {}

void StopSourceIntakeActionCommand::Execute() {
  m_shooterSubsystem->StopMotors();
  m_shooterSubsystem->MoveFeeder(0);
}

bool StopSourceIntakeActionCommand::IsFinished() { return true; }
