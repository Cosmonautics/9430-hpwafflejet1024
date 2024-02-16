#include "subsystems/Shooter.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"

using namespace ShooterConstants;

Shooter::Shooter() {
  m_pivotPIDController.SetP(kP);
  m_pivotPIDController.SetI(kI);
  m_pivotPIDController.SetD(kD);
}

void Shooter::PivotToSetPoint(double setPoint) {
  double currentAngle =m_pivotEncoder.GetPosition() * (180.0 / M_PI);
  if (IsTargetInRestrictedRange(setPoint) ||
      IsTargetInRestrictedRange(currentAngle)) {
    auto exitAbove145 = (currentAngle <= 145.0) ||
                        (currentAngle > 145.0 && setPoint < currentAngle);
    if (exitAbove145) {
      setPoint = 145.0 + 1.0;
    } else {
      setPoint = 0.0;
    }
  }
  m_targetSetpoint = units::degree_t(setPoint);
  double targetPosition = setPoint * (M_PI / 180.0);
  m_pivotPIDController.SetReference(targetPosition,
                                    rev::ControlType::kPosition);
}

bool Shooter::IsTargetInRestrictedRange(double target) {
  return target >= 1.0 && target <= 145.0;
}

bool Shooter::IsAtSetPoint() {
  auto currentAngle =
      units::degree_t(m_pivotEncoder.GetPosition() * (180.0 / M_PI));
  // Assuming you have a method or a way to get the target setpoint for
  // comparison and a tolerance for how close to the setpoint is acceptable
  constexpr auto tolerance = 1_deg;  // Specify an appropriate tolerance

  // Calculate the difference between the current angle and the target setpoint
  auto error = std::abs((currentAngle - m_targetSetpoint).to<double>());

  // Check if the current position is within the tolerance of the target
  // setpoint
  return error <= tolerance.to<double>();
}

void Shooter::ShootMotors(bool isPressed, double speed) {
  if (isPressed) {
    m_shooterMotorLeft.Set(-speed);
    m_shooterMotorRight.Set(speed * 0.75);
  } else {
    m_shooterMotorLeft.Set(0);
    m_shooterMotorRight.Set(0);
  }
}

void Shooter::ShooterDropNote(bool isPressed, double speed) {
  double dropSpeed = 0.10;  // drop positive speed

  if (isPressed) {
    m_shooterMotorLeft.Set(-dropSpeed);
    m_shooterMotorRight.Set(dropSpeed);
  } else {
    m_shooterMotorLeft.Set(0);
    m_shooterMotorRight.Set(0);
  }
}

void Shooter::ShooterPickUpNote(bool isPressed, double speed) {
  double pickUpSpeed = -0.10;  // pick up is negative speed

  if (isPressed) {
    m_shooterMotorLeft.Set(-pickUpSpeed);
    m_shooterMotorRight.Set(pickUpSpeed);
  } else {
    m_shooterMotorLeft.Set(0);
    m_shooterMotorRight.Set(0);
  }
}