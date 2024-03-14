#pragma once

#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "utils/LimelightHelpers.h"

class Limelight : public frc2::Subsystem {
 public:
  Limelight();
  void Periodic() override;

  bool HasTarget(); // use LimelightHelpers::getTV() if == 1.0
  double GetTargetX(); // use LimelightHelpers::getTX()
  double GetTargetY(); // use LimelightHelpers::getTY()
  double GetTargetArea(); // use LimelightHelpers::getTA()

  void SetLEDOn(); // use LimelightHelpers::setLEDMode_ForceOn()
  void SetLEDOff(); // use LimelightHelpers::setLEDMode_ForceOff
  void SetLEDBlink(); // use LimelightHelpers::setLEDMode_ForceBlink()
  double CalculateDistanceToTarget(); // LimelightHelpers::getBotPoseEstimate() might be useful here as well as get*pose() methods. 

 private:
  std::shared_ptr<nt::NetworkTable> limelightTable;

  // network table key values
  const std::string tv = "tv";
  const std::string tx = "tx";
  const std::string ty = "ty";
  const std::string ta = "ta";
  const std::string ledMode = "ledMode";
};
