#pragma once 

#include <rev/CANSparkFlex.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "Elevator.h"

// one pivot motor - shooter manipulator
// one roller motor - for intake part (will function at shooting out at full speed)
// two shooter motor - slow, "fly" wheels

class Shooter {
 public:
  Shooter();


  void ShootMotors(bool isPressed, double speed);

  void PickUpNote(bool isPressed, double speed);

  void DropNote(bool isPressed, double speed);

 private:
  rev::CANSparkFlex m_shooterMotorLeft{
      DriveConstants::kShooterLeftCanId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkFlex m_shooterMotorRight{
      DriveConstants::kShooterRightCanId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
};