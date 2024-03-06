#ifndef STOP_INTAKE_MOTOR_COMMAND_H
#define STOP_INTAKE_MOTOR_COMMAND_H  // rename these to the name of the command
                                     // in all caps followed by _H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Intake.h"  // import all relevant subsystems

class StopIntakeMotorCommand
    : public frc2::CommandHelper<frc2::Command, StopIntakeMotorCommand> {
 public:
  StopIntakeMotorCommand(
      Intake* intakeSubsystem);  // Put in all relevant subsystems

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;

 private:
  Intake* m_intakeSubsystem;
  bool cmdFinished;
};

#endif  // STOP_INTAKE_MOTOR_COMMAND_H // (replace this with the name of the
        // command name)