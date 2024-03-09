#ifndef MEC_H
#define MEC_H  // rename these to the name of the command in all
               // caps followed by _H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Elevator.h"  // import all relevant subsystems

class SetElevatorBrakeCommand
    : public frc2::CommandHelper<frc2::Command, SetElevatorBrakeCommand> {
 public:
  SetElevatorBrakeCommand(Elevator* elevatorSubsystem,
                      bool brake);  // Put in all relevant subsystems

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;

 private:
  Elevator* m_elevatorSubsystem;
  bool m_brake;
};

#endif  // COMMAND_TEMPLATE_H // (replace this with the name of the command
        // name)

// Do ONE thing, in each command