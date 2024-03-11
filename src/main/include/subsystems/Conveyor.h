#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

using namespace ConveyorConstants;

class Conveyor : public frc2::SubsystemBase {
 public:
  Conveyor();
  void Periodic() override;
  void Forward();
  void Reverse();
  void Stop();
  void Move(double speed);

 private:
  rev::CANSparkMax conveyorMotor{kConveyorCanId,
                                 rev::CANSparkMax::MotorType::kBrushless};
  frc::DigitalInput limitSwitch{kLimitSwitchChannel};
};
