#pragma once

#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class Limelight : public frc2::Subsystem {
 public:
  Limelight();
  void Periodic() override;

  bool HasTarget();
  double GetTargetX();
  double GetTargetY();
  double GetTargetArea();

  void SetLEDOn();
  void SetLEDOff();
  void SetLEDBlink();
  double CalculateDistanceToTarget();

 private:
  std::shared_ptr<nt::NetworkTable> limelightTable;

  // network table key values
  const std::string tv = "tv";
  const std::string tx = "tx";
  const std::string ty = "ty";
  const std::string ta = "ta";
  const std::string ledMode = "ledMode";
};
