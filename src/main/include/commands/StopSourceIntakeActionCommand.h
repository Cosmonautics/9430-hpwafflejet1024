#ifndef STOP_SOURCE_INTAKE_ACTION_COMMAND_H
#define STOP_SOURCE_INTAKE_ACTION_COMMAND_H  // rename these to the name of the
                                           // command in all
                                           // caps followed by _H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Elevator.h"  // import all relevant subsystems
#include "subsystems/Shooter.h"   // import all relevant subsystems

class StopSourceIntakeActionCommand
    : public frc2::CommandHelper<frc2::Command, StopSourceIntakeActionCommand> {
 public:
  StopSourceIntakeActionCommand(
      Elevator* elevatorSubsystem,
      Shooter* shooterSubsystem);  // Put in all relevant subsystems

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;

 private:
  Elevator*
      m_elevatorSubsystem;  // Put all relevant subsystems here as pointers
  Shooter* m_shooterSubsystem;
};

#endif  // STOP_SOURCE_INTAKE_ACTION_COMMAND_H // (replace this with the name of
        // the command name)