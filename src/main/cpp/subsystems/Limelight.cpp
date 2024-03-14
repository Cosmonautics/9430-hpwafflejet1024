#include "subsystems/Limelight.h"

Limelight::Limelight() {
  limelightTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

void Limelight::Periodic() {
  // perdiodic stuff if neccessary
}

bool Limelight::HasTarget() {
  return limelightTable->GetNumber(tv, 0.0) == 1.0;
}

double Limelight::GetTargetX() {
  return limelightTable->GetNumber(tx, 0.0);
}

double Limelight::GetTargetY() {
  return limelightTable->GetNumber(ty, 0.0);
}

double Limelight::GetTargetArea() {
  return limelightTable->GetNumber(ta, 0.0);
}

void Limelight::SetLEDOn() { limelightTable->PutNumber(ledMode, 3); }

void Limelight::SetLEDOff() { limelightTable->PutNumber(ledMode, 1); }

void Limelight::SetLEDBlink() {
  limelightTable->PutNumber(ledMode, 2);
}
double Limelight::CalculateDistanceToTarget() {
  double targetOffsetAngle_Vertical = limelightTable->GetNumber(ty, 0.0);
  double limelightMountAngleDegrees = 25.0;  // Customize based on your setup
  double limelightLensHeightInches = 20.0;   // Customize based on your setup
  double goalHeightInches = 60.0;            // Customize based on your setup

  double angleToGoalDegrees =
      limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  double angleToGoalRadians = angleToGoalDegrees * (M_PI / 180.0);
  return (goalHeightInches - limelightLensHeightInches) /
         tan(angleToGoalRadians);
}