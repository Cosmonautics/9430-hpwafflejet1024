#ifndef COMMAND_TEMPLATE_H
#define COMMAND_TEMPLATE_H  // rename these to the name of the command in all
                            // caps followed by _H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Intake.h"  // import all relevant subsystems

class MoveIntakeWheelsCommand
    : public frc2::CommandHelper<frc2::Command, MoveIntakeWheelsCommand> {
 public:
  MoveIntakeWheelsCommand(Intake* intakeSubsystem, double speed,
                          bool pressed);  // Put in all relevant subsystems

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;

 private:
  Intake* m_intakeSubsystem;
  double m_speed;
  bool m_pressed;
};

#endif  // COMMAND_TEMPLATE_H // (replace this with the name of the command
        // name)

// Do ONE thing, in each command