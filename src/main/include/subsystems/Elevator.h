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
      // Adjusted to associate the encoder with the right motor
      m_ElevatorMotorRight.GetAbsoluteEncoder(
          rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)};
  frc::PIDController m_pidController{kP, kI, kD};
  double currentPositionInches = 0;  // Current elevator position in inches
  double targetPositionInches = 0;   // Target elevator position in inches
  // trigger: xbox controller button

  // go down
  // trigger: xbox controller button

  // throughbore encoder
  // 2 motors, one inverted

  // Helper methods
  void UpdatePosition();
  double ConvertInchesToEncoderUnits(double inches);
  double ConvertEncoderUnitsToInches(double units);
  void Move(double speed);
};
