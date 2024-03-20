#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"
#include "subsystems/Limelight.h"

class DoAlignShooterWithAprilTagCommand
    : public frc2::CommandHelper<frc2::Command, DoAlignShooterWithAprilTagCommand> {
 public:
  DoAlignShooterWithAprilTagCommand(Shooter* shooter, Limelight* limelight);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  double CalculateRotationSpeed(double targetOffsetAngle);

 private:
  Shooter* m_shooter;
  Limelight* m_limelight;
  double m_targetOffsetAngleHorizontal;
  bool m_isAligned;
};
