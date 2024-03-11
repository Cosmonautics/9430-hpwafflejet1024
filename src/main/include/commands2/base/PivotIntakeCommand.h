#ifndef COMMAND_TEMPLATE_H
#define COMMAND_TEMPLATE_H  // rename these to the name of the command in all
                            // caps followed by _H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Intake.h"  // import all relevant subsystems

class PivotIntakeCommand
    : public frc2::CommandHelper<frc2::Command, PivotIntakeCommand> {
 public:
  PivotIntakeCommand(Intake* intakeSubsystem, double position,
                     bool down);  // Put in all relevant subsystems

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;

 private:
  Intake* m_intakeSubsystem;
  double m_position;
  bool m_down;
};

#endif  // COMMAND_TEMPLATE_H // (replace this with the name of the command
        // name)

// Do ONE thing, in each command