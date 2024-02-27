#include "commands/DoClimbActionCommand.h"  // include command header

DoClimbActionCommand::DoClimbActionCommand(Elevator* elevatorSubsystem,
                                           Shooter* shooterSubsystem,
                                           bool isClimb1)
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_isClimb1(isClimb1) {
  AddRequirements({elevatorSubsystem, shooterSubsystem});
}

void DoClimbActionCommand::Initialize() {
  cmdFinished = false;
  timer = new frc::Timer();
  timer->Reset();
}

void DoClimbActionCommand::Execute() {
  if (m_isClimb1) {
    m_elevatorSubsystem->MoveToPosition(
        PositionConstants::kElevatorShooterPosition);
    m_shooterSubsystem->PivotToSetPoint(
        PositionConstants::kShooterShooterPosition);
    while (!timer->HasElapsed(1_s)) {
      //wait / do nothing
    }
    m_shooterSubsystem->ShootMotors(true, 0.30);
    m_shooterSubsystem->MoveFeeder(-0.50);
    while (!timer->HasElapsed(3_s)) {
      //wait / do nothing
    }
    m_shooterSubsystem->MoveFeeder(0);
    m_shooterSubsystem->ShootMotors(false, 0);
  }

  // Execute when Y is pressed; if Y is pressed again, stop the climb sequence
  // Set EL brake piston to lock position INGORE For NOW
  // Move elevator to climb position (SM carriage to the top)
  // Move shooter to trap position (thigh highs)
  // Wait for elevator to get done moving
  // Set shooter to AMP scoring mode (TBD may be tweaked)
  // basically, repeat the same code in AMP scoring mode.
  // Set shooter trap piston to score position
  // Set shooter feeder motor to shoot out at ~50%
  // Wait for ~2 seconds
  // Set shooter feeder motor 0%
  // Set shooter motor 0%
  cmdFinished = true;  // should only conditionally set to true if and only if
                       // tests for motor states return true
}

bool DoClimbActionCommand::IsFinished() { return cmdFinished; }

// tests should go here to set target states for completion to return true
// should only conditionally set to true if and only if tests for motor states
// return true finished logic should ensure test conditions pass (use motor
// position/motor state methods)
