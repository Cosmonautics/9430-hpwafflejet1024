#include "commands/DoSpeakerScoreActionCommand.h" 

DoSpeakerScoreActionCommand::DoSpeakerScoreActionCommand( 
    Elevator* elevatorSubsystem, Shooter* shooterSubsystem) // add all the relevant subsystems to constructor 
    : m_elevatorSubsystem(elevatorSubsystem),
      m_shooterSubsystem(shooterSubsystem) {
  AddRequirements({elevatorSubsystem, shooterSubsystem});
}

void DoSpeakerScoreActionCommand::Initialize() {}

void DoSpeakerScoreActionCommand::Execute() {
  // Check if AMP Score Position status == TRUE // (If it's already set here, it won't move; if not, set it to amp score position)
  m_shooterSubsystem->PivotToSetPoint(
      PositionConstants::kShooterShooterPosition);
  frc2::WaitCommand(0.8_s).Schedule();
  m_elevatorSubsystem->MoveToPosition(
      PositionConstants::kElevatorShooterPosition);
    // Set shooter motor 100%
  frc2::WaitCommand(0.2_s).Schedule();
  m_shooterSubsystem->ShootMotors(true, 1.0);
    // (TBD VISION) auto align DT to point at the speaker
    // (TBD VISION/PATHING) pivot shooter manipulator to proper angle based on distance from goal
  
    // Set shooter feeder motor 100% 
  m_shooterSubsystem->MoveFeeder(1.0);
      // Wait ~1 second 
  frc2::WaitCommand(1_s).Schedule();

    // Set shooter motor 0%
  m_shooterSubsystem->ShootMotors(false, 0.0);
    // Set shooter feeder motor 0% 
  m_shooterSubsystem->MoveFeeder(0.0);

}

bool DoSpeakerScoreActionCommand::IsFinished() {
  // Return code here
  // return ...;
}
