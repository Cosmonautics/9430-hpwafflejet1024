#ifndef COMMAND_TEMPLATE_H
#define COMMAND_TEMPLATE_H  // rename these to the name of the command in all
                            // caps followed by _H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Shooter.h"  // import all relevant subsystems

class MoveShooterFeederWheelsCommand
    : public frc2::CommandHelper<frc2::Command, MoveShooterFeederWheelsCommand> {
 public:
  MoveShooterFeederWheelsCommand(Shooter* shooterSubsystem,
                           double speed);  // Put in all relevant subsystems

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;

 private:
  Shooter* m_shooterSubsystem;
  double m_speed;
};

#endif  // COMMAND_TEMPLATE_H // (replace this with the name of the command
        // name)

// Do ONE thing in each command