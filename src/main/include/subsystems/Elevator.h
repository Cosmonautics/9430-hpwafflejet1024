#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <rev/CanSparkMax.h>

#include "Constants.h"

using namespace ElevatorConstants;

class Elevator : public frc2::Subsystem {
 public:
  Elevator();
  void Periodic() override;
  void MoveToPosition(double position);
  void ConfigureMotors();
  bool AtTargetPosition() const;

 private:
  rev::CANSparkMax m_ElevatorMotorLeft{
      kElevatorLeftCanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_ElevatorMotorRight{
      kElevatorRightCanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::SparkAbsoluteEncoder m_ElevatorEncoder{
      m_ElevatorMotorLeft.GetAbsoluteEncoder(
          rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)};
  frc::PIDController m_pidController{kP, kI, kD};
  double currentPositionCm =
      0;  // Current elevator position in centimeters
          // Elevator(rev::CANSparkMax m_ElevatorMoterLeft);
  double targetPositionCm = 0;  // Add this to track the target position
  // go up
  // trigger: xbox controller button

  // go down
  // trigger: xbox controller button

  // throughbore encoder
  // 2 motors, one inverted

  // Helper methods
  void UpdatePosition();
  double ConvertCmToEncoderUnits(double cm);
  double ConvertEncoderUnitsToCm(double units);
  void Move(double speed);
};