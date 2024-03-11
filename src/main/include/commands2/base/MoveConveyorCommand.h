#ifndef MCC_H
#define MCC_H  // rename these to the name of the command in all
                            // caps followed by _H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Conveyor.h"  // import all relevant subsystems

class MoveConveyorCommand
    : public frc2::CommandHelper<frc2::Command, MoveConveyorCommand> {
 public:
  MoveConveyorCommand(Conveyor* conveyorSubsystem,
                     double speed);  // Put in all relevant subsystems

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;

 private:
  Conveyor* m_conveyorSubsystem;
  double m_speed;
};

#endif  // COMMAND_TEMPLATE_H // (replace this with the name of the command
        // name)

// Do ONE thing, in each command