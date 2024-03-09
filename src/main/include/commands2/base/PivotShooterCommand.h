#ifndef PSC_H
#define PSC_H  // rename these to the name of the command in all
                            // caps followed by _H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Shooter.h"  // import all relevant subsystems

class PivotShooterCommand
    : public frc2::CommandHelper<frc2::Command, PivotShooterCommand> {
 public:
  PivotShooterCommand(Shooter* shooterSubsystem,
                     double position);  // Put in all relevant subsystems

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;

 private:
  Shooter* m_shooterSubsystem;
  double m_position;
};

#endif  // COMMAND_TEMPLATE_H // (replace this with the name of the command
        // name)

// Do ONE thing in each command