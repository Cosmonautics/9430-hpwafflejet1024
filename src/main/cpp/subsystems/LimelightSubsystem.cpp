#include "subsystems/LimelightSubsystem.h"

LimelightSubsystem::LimelightSubsystem() {
  limelightTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

void LimelightSubsystem::Periodic() {
    //perdiodic stuff if neccessary
}

bool LimelightSubsystem::HasTarget() {
  return limelightTable->GetNumber(tv, 0.0) == 1.0;
}

double LimelightSubsystem::GetTargetX() {
  return limelightTable->GetNumber(tx, 0.0);
}

double LimelightSubsystem::GetTargetY() {
  return limelightTable->GetNumber(ty, 0.0);
}

double LimelightSubsystem::GetTargetArea() {
  return limelightTable->GetNumber(ta, 0.0);
}

void LimelightSubsystem::SetLEDOn() { limelightTable->PutNumber(ledMode, 3); }

void LimelightSubsystem::SetLEDOff() { limelightTable->PutNumber(ledMode, 1); }

void LimelightSubsystem::SetLEDBlink() {
  limelightTable->PutNumber(ledMode, 2);
}
