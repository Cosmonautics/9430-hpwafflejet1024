#ifndef SDTC_H
#define SDTC_H  // rename these to the name of the command in all
                            // caps followed by _H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/DriveSubsystem.h"  // import all relevant subsystems

class StopDriveTrainCommand
    : public frc2::CommandHelper<frc2::Command, StopDriveTrainCommand> {
 public:
  StopDriveTrainCommand(DriveSubsystem* driveSubsystem);  // Put in all relevant subsystems

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;

 private:
  DriveSubsystem* m_driveSubsystem;
};

#endif  // COMMAND_TEMPLATE_H // (replace this with the name of the command
        // name)

// Do ONE thing, in each command