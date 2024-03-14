#pragma once

#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>  // FLAG: may need to delete (incl in LimelightHelpers.h)
#include <networktables/NetworkTableInstance.h>  // FLAG: may need to delete (incl in LimelightHelpers.h)

#include "utils/LimelightHelpers.h"

class Limelight : public frc2::Subsystem {
 public:
  Limelight();
  void Periodic() override;

  bool HasTarget();        // use LimelightHelpers::getTV() if == 1.0
  double GetTargetX();     // use LimelightHelpers::getTX()
  double GetTargetY();     // use LimelightHelpers::getTY()
  double GetTargetArea();  // use LimelightHelpers::getTA()

  void SetLEDOn();     // use LimelightHelpers::setLEDMode_ForceOn()
  void SetLEDOff();    // use LimelightHelpers::setLEDMode_ForceOff
  void SetLEDBlink();  // use LimelightHelpers::setLEDMode_ForceBlink()
  double CalculateDistanceToTarget(
      bool isRed);  // LimelightHelpers::getBotPoseEstimate() might
                    // be useful here as well as get*pose() methods.
};
