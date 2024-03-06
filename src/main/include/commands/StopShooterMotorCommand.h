#ifndef STOP_SHOOTER_MOTOR_COMMAND_H
#define STOP_SHOOTER_MOTOR_COMMAND_H  // rename these to the name of the command
                                     // in all caps followed by _H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Shooter.h"  // import all relevant subsystems

class StopShooterMotorCommand
    : public frc2::CommandHelper<frc2::Command, StopShooterMotorCommand> {
 public:
  StopShooterMotorCommand(
      Shooter* shooterSubsystem);  // Put in all relevant subsystems

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;

 private:
  Shooter* m_shooterSubsystem;
  bool cmdFinished;
};

#endif  // STOP_SHOOTER_MOTOR_COMMAND_H // (replace this with the name of the
        // command name)