#include "subsystems/Intake.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"

Intake::Intake() : m_intakePivotMax(DriveConstants::kIntakeLeftCanId,rev::CANSparkMaxLowLevel::MotorType::kBrushless), m_intakeRollersFlex(DriveConstants::kIntakeLeftCanId,rev::CANSparkLowLevel::MotorType::kBrushless) {

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