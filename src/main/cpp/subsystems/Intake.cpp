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

void Intake::PivotToAngle(double intakeAngleDegrees) {

  double intakeAngleRadians = intakeAngleDegrees * (M_PI / 180.0);

  intakeAngleRadians = fmod(intakeAngleRadians, 2 * M_PI);
  if (intakeAngleRadians < 0) {
    intakeAngleRadians += 2 * M_PI;  
  }

  double intakeAngleRotations = intakeAngleRadians / (2 * M_PI);

  const double minAngleRotations = 3.0 / 4.0;  
  const double maxAngleRotations = 1.0;        

  if (intakeAngleRotations < minAngleRotations) {
    intakeAngleRotations = minAngleRotations;
  } else if (intakeAngleRotations > maxAngleRotations) {

    intakeAngleRotations = maxAngleRotations;
  }

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