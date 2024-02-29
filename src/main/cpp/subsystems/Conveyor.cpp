// ConveyorSubsystem.cpp

#include "subsystems/Conveyor.h"

Conveyor::Conveyor() {
  conveyorMotor.SetInverted(true);
  conveyorMotor.SetSmartCurrentLimit(40);
  conveyorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Conveyor::Periodic() {
  // if (limitSwitch.Get() == true) {
  // Stop();
  //}
}

void Conveyor::Forward() {
  // if (limitSwitch.Get() == false) {
  conveyorMotor.Set(1);
  //}
}

void Conveyor::Reverse() { conveyorMotor.Set(-1); } // opening braces should go at end of line, close brace should be on new line

void Conveyor::Stop() { conveyorMotor.Set(0); } // same thing here 
