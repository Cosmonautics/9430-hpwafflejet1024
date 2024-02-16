// RunConveyorCommand.h

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Conveyor.h"

class RunConveyorCommand
    : public frc2::CommandHelper<frc2::Command, RunConveyorCommand> {
 public:
  explicit RunConveyorCommand(Conveyor* subsystem, bool forward);
  void Initialize() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  void Execute() override;

 private:
  Conveyor* m_conveyorSubsystem;
  bool m_forward;
};
