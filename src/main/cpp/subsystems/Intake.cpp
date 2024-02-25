#include "subsystems/Intake.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"

/*
Todo:
Pivot: have intake motor pivot to a floor position
elevator postion and shooter position remain defult to transit positon



*/

Intake::Intake() {
  m_pidController.SetFeedbackDevice(m_intakePivotAbsoluteEncoder);
  m_pidController.SetP(IntakeConstants::kP);
  m_pidController.SetI(IntakeConstants::kI);
  m_pidController.SetD(IntakeConstants::kD);
}

void Intake::IntakeDropNote(bool isPressed, double speed) {
  double dropSpeed = 0.10;  // drop positive speed

  if (isPressed) {
    m_intakeMotorLeft.Set(-dropSpeed);
    m_intakeMotorRight.Set(dropSpeed);
  } else {
    m_intakeMotorLeft.Set(0);
    m_intakeMotorRight.Set(0);
  }
}

void Intake::PivotToAngle(double intakeAngleRotations) {
  // Define minimum and maximum rotation limits within the 0 to 1 range
  const double minAngleRotations =
      0.0;  // Example minimum limit, adjust as needed
  const double maxAngleRotations =
      0.3;  // Example maximum limit, adjust as needed

  // Clamp the input to ensure it's within the specified limits
  if (intakeAngleRotations < minAngleRotations) {
    intakeAngleRotations = minAngleRotations;
  } else if (intakeAngleRotations > maxAngleRotations) {
    intakeAngleRotations = maxAngleRotations;
  }

  m_targetSetpointRotations = intakeAngleRotations;
  // Set the target position using PID controller
  m_pidController.SetReference(intakeAngleRotations,
                               rev::ControlType::kPosition);
}

void Intake::IntakePickUpNote(bool isPressed, double speed) {
  double pickUpSpeed = -0.10;  // pick up is negative speed

  if (isPressed) {
    m_intakeMotorLeft.Set(-pickUpSpeed);
    m_intakeMotorRight.Set(pickUpSpeed);
  } else {
    m_intakeMotorLeft.Set(0);
    m_intakeMotorRight.Set(0);
  }
}

void Intake::ControlIntakeMotors(bool isPressed, double speed) {
  if (isPressed) {
    m_intakeMotorLeft.Set(-speed);
    m_intakeMotorRight.Set(speed);
  } else {
    m_intakeMotorLeft.Set(0);
    m_intakeMotorRight.Set(0);
  }
}

bool Intake::IsAtSetPoint() {
  double currentAngleRotations = m_intakePivotAbsoluteEncoder.GetPosition();
  constexpr double toleranceRotations = 1.0 / 360.0;  // Tolerance in rotations

  double error = std::abs(currentAngleRotations - m_targetSetpointRotations);

  return error <= toleranceRotations;
}