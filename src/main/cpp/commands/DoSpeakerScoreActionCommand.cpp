#include "commands/DoSpeakerScoreActionCommand.h"

DoSpeakerScoreActionCommand::DoSpeakerScoreActionCommand(
    Elevator* elevatorSubsystem,
    Shooter*
        shooterSubsystem)  // add all the relevant subsystems to constructor
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem});
}

void DoSpeakerScoreActionCommand::Initialize() {
  cmdFinished = false;  // tests should go here to set target states for
                        // completion to return true
  timer = new frc::Timer();
  timer->Reset();
}

void DoSpeakerScoreActionCommand::Execute() {
  timer->Start();
  m_shooterSubsystem->ShootMotors(true, -1.0);

  // Check if AMP Score Position status == TRUE // (If it's already set here, it
  // won't move; if not, set it to amp score position) this code was copied from
  // MoveToAMPSpeakerScorePositionCommand.cpp as an alternative to calling the
  // command from this command. (which is probably not possible or bad design)
  m_shooterSubsystem->InvertMotor(true);
  m_shooterSubsystem->PivotToSetPoint(
      PositionConstants::kShooterShooterPosition);
  // frc2::WaitCommand(0.8_s).Schedule();
  m_elevatorSubsystem->MoveToPosition(
      PositionConstants::kElevatorShooterPosition, false);

  // Set shooter motor 100%
  // frc2::WaitCommand(0.2_s).Schedule();

  // (TBD VISION) auto align DT to point at the speaker
  // (TBD VISION/PATHING) pivot shooter manipulator to proper angle based on
  // distance from goal
  while (!timer->HasElapsed(3_s)) {
    // After 2 seconds, move the feeder and mark the command as complete
  }
  m_shooterSubsystem->MoveFeeder(-1.0);  // Set shooter feeder motor 100%
  while (!timer->HasElapsed(4_s)) {
    // After 2 seconds, move the feeder and mark the command as complete
  }
  m_shooterSubsystem->StopMotors();
  m_shooterSubsystem->MoveFeeder(0);
  timer->Stop();
  cmdFinished = true;
  // frc2::WaitCommand(1_s).Schedule(); // Wait ~1 second

  // m_shooterSubsystem->ShootMotors(false, 0.0); // Set shooter motor 0%

  // m_shooterSubsystem->MoveFeeder(0.0); // Set shooter feeder motor 0%
}

bool DoSpeakerScoreActionCommand::IsFinished() {
  return cmdFinished;  // finished logic should ensure test conditions pass (use
                       // motor position/motor state methods)
}
