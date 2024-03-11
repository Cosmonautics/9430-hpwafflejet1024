#include "commands2/DoClimb1Command.h"

using namespace PositionConstants;

DoClimb1Command::DoClimb1Command(Elevator* elevatorSubsystem,
                                 Shooter* shooterSubsystem,
                                 Intake* intakeSubsystem) {
  AddCommands(
      PivotShooterCommand(shooterSubsystem, kShooterClimb1Position),
      frc2::WaitCommand(0.5_s),
      MoveElevatorCommand(elevatorSubsystem, kElevatorClimb1Position, false),
      PivotIntakeCommand(intakeSubsystem, kIntakeClimb1Position, true));
}