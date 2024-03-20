#include "subsystems/Limelight.h"

Limelight::Limelight() {}

void Limelight::Periodic() {
  // perdiodic stuff if neccessary
}

bool Limelight::HasTarget() { return LimelightHelpers::getTV() == 1.0; }

double Limelight::GetTargetX() { return LimelightHelpers::getTX(); }

double Limelight::GetTargetY() { return LimelightHelpers::getTY(); }

double Limelight::GetTargetArea() { return LimelightHelpers::getTA(); }

void Limelight::SetLEDOn() { LimelightHelpers::setLEDMode_ForceOn(); }

void Limelight::SetLEDOff() { LimelightHelpers::setLEDMode_ForceOff(); }

void Limelight::SetLEDBlink() { LimelightHelpers::setLEDMode_ForceBlink(); }

double Limelight::CalculateDistanceToTarget(bool isRed) {
  double limelightMountAngleDegrees = 30.0;

  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 5.0;

  // distance from the target to the floor
  double goalHeightInches = 57.5;

  double angleToGoalDegrees =
      limelightMountAngleDegrees + LimelightHelpers::getTY();
  double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

  // calculate distance
  return (goalHeightInches - limelightLensHeightInches) /
         tan(angleToGoalRadians);
}