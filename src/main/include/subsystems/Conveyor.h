#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

#include "Constants.h"

using namespace ConveyorConstants;

class Conveyor : public frc2::SubsystemBase {
 public:
  Conveyor();
  void Periodic() override;
  void SimulationPeriodic() override; 
  void Forward();
  void Reverse();
  void Stop();

 private:
  rev::CANSparkMax conveyorMotor{kConveyorCanId, rev::CANSparkMax::MotorType::kBrushless};
  frc::DigitalInput limitSwitch{kLimitSwitchChannel};
};
