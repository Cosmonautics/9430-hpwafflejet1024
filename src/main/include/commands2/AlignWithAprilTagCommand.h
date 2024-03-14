#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/Limelight.h"

class AlignWithAprilTagCommand
    : public frc2::CommandHelper<frc2::Command, AlignWithAprilTagCommand> {
 public:
  AlignWithAprilTagCommand(DriveSubsystem* drive, Limelight* limelight);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  double CalculateRotationSpeed(double targetOffsetAngle);

 private:
  DriveSubsystem* m_drive;
  Limelight* m_limelight;
  double m_targetOffsetAngleHorizontal;
  bool m_isAligned;
};
