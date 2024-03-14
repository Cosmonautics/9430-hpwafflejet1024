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
  return isRed ? LimelightHelpers::getBotPoseEstimate_wpiRed().avgTagDist
               : LimelightHelpers::getBotPoseEstimate_wpiBlue().avgTagDist;
}