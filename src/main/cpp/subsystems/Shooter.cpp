#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

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