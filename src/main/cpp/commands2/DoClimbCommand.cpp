#include "commands2/DoClimbCommand.h"

using namespace PositionConstants;

DoClimbCommand::DoClimbCommand(Elevator* elevatorSubsystem) {
  AddCommands(
      MoveElevatorCommand(elevatorSubsystem, kElevatorClimbPosition, true),
      frc2::WaitCommand(3.5_s),
      SetElevatorBrakeCommand(elevatorSubsystem, true));
}