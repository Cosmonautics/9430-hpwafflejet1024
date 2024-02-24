#include "subsystems/Shooter.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"

using namespace ShooterConstants;

Shooter::Shooter() {
  m_pivotPIDController.SetFeedbackDevice(m_pivotEncoder);
  m_pivotPIDController.SetP(kP);
  m_pivotPIDController.SetI(kI);
  m_pivotPIDController.SetD(kD);
}

void Shooter::PivotToSetPoint(double setPointRotations) {
  // Ensure setPointRotations is in the 0-1 range, representing a full rotation
  double currentAngleRotations =
      m_pivotEncoder
          .GetPosition();  // Assuming this returns rotations in 0-1 range

  if (IsTargetInRestrictedRange(currentAngleRotations) ||
      IsTargetInRestrictedRange(setPointRotations)) {
    bool exitAboveRestrictedRange =
        (currentAngleRotations <= (145.0 / 360.0)) ||
        (currentAngleRotations > (145.0 / 360.0) &&
         setPointRotations < currentAngleRotations);
    if (exitAboveRestrictedRange) {
      setPointRotations =
          (145.0 + 1.0) /
          360.0;  // Adjust setpoint to avoid restricted range, in rotations
    } else {
      setPointRotations = 0.0;  // Reset setpoint if necessary
    }
  }

  m_targetSetpointRotations =
      setPointRotations;  // Store the setpoint in rotations for error
                          // calculation

  // Set the PID reference using rotations directly
  m_pivotPIDController.SetReference(setPointRotations,
                                    rev::ControlType::kPosition);
}

bool Shooter::IsTargetInRestrictedRange(double targetRotations) {
  // Check if the target (in rotations) is within the restricted range
  return targetRotations >= (1.0 / 360.0) && targetRotations <= (145.0 / 360.0);
}

bool Shooter::IsAtSetPoint() {
  // Get current angle in rotations for comparison
  double currentAngleRotations = m_pivotEncoder.GetPosition();
  constexpr double toleranceRotations = 1.0 / 360.0;  // Tolerance in rotations

  // Calculate the difference between the current angle and the target setpoint
  // in rotations
  double error = std::abs(currentAngleRotations - m_targetSetpointRotations);

  // Check if the current position is within the tolerance of the target
  // setpoint
  return error <= toleranceRotations;
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