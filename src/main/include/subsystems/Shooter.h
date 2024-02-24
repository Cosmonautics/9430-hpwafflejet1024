#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <rev/CANSparkFlex.h>

#include "Constants.h"

// one pivot motor - shooter manipulator
// one roller motor - for intake part (will function at shooting out at full
// speed) two shooter motor - slow, "fly" wheels

class Shooter : public frc2::Subsystem {
 public:
  Shooter();

  void ShootMotors(bool isPressed, double speed);

  void ShooterPickUpNote(bool isPressed, double speed);

  void ShooterDropNote(bool isPressed, double speed);

  bool IsTargetInRestrictedRange(double target);
  void PivotToSetPoint(double setPoint);
  bool IsAtSetPoint();
  void ManualMove(double speed);
  bool ToggleManualOverride();

 private:
  rev::CANSparkFlex m_shooterMotorLeft{
      ShooterConstants::kShooterLeftCanId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkFlex m_shooterMotorRight{
      ShooterConstants::kShooterRightCanId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_pivotMotor{ShooterConstants::kShooterPivotCanId,
                                 rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkAbsoluteEncoder m_pivotEncoder{m_pivotMotor.GetAbsoluteEncoder(
      rev::SparkAbsoluteEncoder::Type::kDutyCycle)};
  rev::SparkMaxPIDController m_pivotPIDController =
      m_pivotMotor.GetPIDController();
  double m_targetSetpointRotations = 0.0;
  bool manualOverride = false;
};