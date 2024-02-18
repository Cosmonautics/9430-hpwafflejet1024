#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>

#include "subsystems/Shooter.h"

class PivotShooterToPositionCommand
    : public frc2::CommandHelper<frc2::Command, PivotShooterToPositionCommand> {
 public:
  PivotShooterToPositionCommand(Shooter* shooter, double setPoint);

  void Initialize() override;
  bool IsFinished() override;
  void Execute() override;
 private:
  Shooter* m_shooter;
  double m_setPoint;
};
