#include "commands/MoveToClimbPos1Command.h"  // include command header

MoveToClimbPos1Command::MoveToClimbPos1Command(Elevator* elevatorSubsystem,
                                               Shooter* shooterSubsystem,
                                               Intake* intakeSubsystem)
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_intakeSubsystem(intakeSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem, intakeSubsystem});
}

void MoveToClimbPos1Command::Initialize() {
  timer = new frc::Timer();
  timer->Reset();
  
}

void MoveToClimbPos1Command::Execute() {
    m_shooterSubsystem->InvertMotor(true);
  m_shooterSubsystem->PivotToSetPoint( // move the shooter
      PositionConstants::kShooterClimb1Position);
  timer->Start(); 
  while (!timer->HasElapsed(1_s)) {
    // after 1 second, move the intake 
  }
  /*m_intakeSubsystem->PivotToAngle(PositionConstants::kIntakeClimb1Position,
                                  false);*/
  while(!timer->HasElapsed(0.5_s)) {
    // after .5 seconds, move the elevator 
  }
  m_elevatorSubsystem->MoveToPosition( // move the elevator 
      PositionConstants::kElevatorClimb1Position, false);
}

bool MoveToClimbPos1Command::IsFinished() {
  return m_elevatorSubsystem->AtTargetPosition() &&
         m_shooterSubsystem->IsAtSetPoint() &&
         m_intakeSubsystem->IsAtSetPoint();
}