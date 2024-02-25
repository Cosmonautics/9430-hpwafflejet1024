// ConveyorSubsystem.cpp

#include "subsystems/Conveyor.h"

Conveyor::Conveyor() {
  conveyorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Conveyor::Periodic() {
 // if (limitSwitch.Get() == true) {
    //Stop();
  //}
}

void Conveyor::Forward() {
 // if (limitSwitch.Get() == false) {
    conveyorMotor.Set(1);
  //}
}

void Conveyor::Reverse() {
  conveyorMotor.Set(-1);  
}

void Conveyor::Stop() { conveyorMotor.Set(0); }
