#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>

#include "subsystems/Shooter.h"

class PivotToPositionCommand
    : public frc2::CommandHelper<frc2::Command, PivotToPositionCommand> {
 public:
  PivotToPositionCommand(Shooter* shooter, units::degree_t setPoint);

  void Initialize() override;
  bool IsFinished() override;
  void Execute() override;
 private:
  Shooter* m_shooter;
  units::degree_t m_setPoint;
};
