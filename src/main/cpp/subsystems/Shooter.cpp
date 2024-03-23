#include "subsystems/Shooter.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"

using namespace ShooterConstants;

Shooter::Shooter() {
  m_pivotMotor.RestoreFactoryDefaults();
  m_pivotMotor.SetInverted(true);
  m_shooterFeeder.SetInverted(true);
  m_pivotMotor.SetSmartCurrentLimit(20);
  m_pivotPIDController.SetOutputRange(-1.0, 1.0);
  m_pivotPIDController.SetFeedbackDevice(m_pivotEncoder);
  m_pivotPIDController.SetP(kP);
  m_pivotPIDController.SetI(kI);
  m_pivotPIDController.SetD(kD);
  m_pivotMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward,
                            ShooterConstants::kShooterForwardSoftLimit);
  m_pivotMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse,
                            ShooterConstants::kShooterReverseSoftLimit);
  m_pivotMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward,
                               true);
  m_pivotMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse,
                               true);
  InitializeDistanceAngleLookup();
  // Initialization of motors, PID controllers, etc.
}

void Shooter::InitializeDistanceAngleLookup() {
  // Populate the distance-angle lookup table with real data
  distanceAngleLookup = {{36.0, 0.846},  {60.0, 0.817}, {84.0, 0.794},
                         {108.0, 0.777}, {132.0, 0.77}, {156.0, 0.763}};
}

void Shooter::PivotToSetPoint(double setPointRotations) {
  // Ensure setPointRotations is in the 0-1 range, representing a full rotation
  double currentAngleRotations =
      m_pivotEncoder
          .GetPosition();  // Assuming this returns rotations in 0-1 range

  /*  if (IsTargetInRestrictedRange(currentAngleRotations) ||
        IsTargetInRestrictedRange(setPointRotations)) {
      bool exitAboveRestrictedRange =
          (currentAngleRotations <= (100.0 / 360.0)) ||
          (currentAngleRotations > (100.0 / 360.0) &&
           setPointRotations < currentAngleRotations);
      if (exitAboveRestrictedRange) {
        setPointRotations =
            (100.0 + 1.0) /
            360.0;  // Adjust setpoint to avoid restricted range, in rotations
      } else {
        setPointRotations = 0.0;  // Reset setpoint if necessary
      }
    }
  */
  m_targetSetpointRotations =
      setPointRotations;  // Store the setpoint in rotations for error
                          // calculation

  // Set the PID reference using rotations directly
  m_pivotPIDController.SetReference(setPointRotations,
                                    rev::ControlType::kPosition);
}

void Shooter::PivotToSetPointAngle(double setPointAngle) {
  // Ensure setPointRotations is in the 0-1 range, representing a full rotation
  m_targetSetpointRotations =
      setPointAngle / 360.0;  // Store the setpoint in rotations for error
                              // calculation

  // Set the PID reference using rotations directly
  m_pivotPIDController.SetReference(setPointAngle / 360.0,
                                    rev::ControlType::kPosition);
}

bool Shooter::IsTargetInRestrictedRange(double targetRotations) {
  // Check if the target (in rotations) is within the restricted range
  return targetRotations >= (1.0 / 360.0) && targetRotations <= (100.0 / 360.0);
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

void Shooter::InvertMotor(bool invert) {  // not clear what motor will be
                                          // inverted by function name alone
  m_pivotMotor.SetInverted(invert);
}

void Shooter::ShooterDropNote(bool isPressed, double speed) {
  double dropSpeed = -speed;  // drop positive speed

  if (isPressed) {
    m_shooterMotorLeft.Set(-dropSpeed);
    m_shooterMotorRight.Set(dropSpeed);
  } else {
    m_shooterMotorLeft.Set(0);
    m_shooterMotorRight.Set(0);
  }
}

void Shooter::MoveFeeder(double speed) {
  m_shooterFeeder.Set(speed);
}  // all braces should start new lines, end braces should end on new lines

void Shooter::ShooterPickUpNote(bool isPressed, double speed) {
  double pickUpSpeed = speed;  // pick up is negative speed

  if (isPressed) {
    m_shooterMotorLeft.Set(
        -pickUpSpeed);  // motorLeft should be inverted from motorRight so you
                        // don't have to negate values
    m_shooterMotorRight.Set(pickUpSpeed);
  } else {
    m_shooterMotorLeft.Set(0);
    m_shooterMotorRight.Set(0);
  }
}

void Shooter::SetAngleBasedOnDistance(double distance, double elevatorHeight) {
  double apriltagHeight = 8.125;
  double apriltagToFloor = 51.875;
  double apriltagToSpeaker = 17.0;  // inches or else
  double targetHeight = (apriltagHeight + apriltagToFloor + apriltagToSpeaker) +
                        elevatorHeight + 5;
  double hyp = sqrt((targetHeight * targetHeight) + (distance * distance));
  double desiredAngle = asin(sin(targetHeight / hyp));
  frc::SmartDashboard::PutNumber("desangle",desiredAngle);
  //PivotToSetPointAngle(desiredAngle);
  // angle theta = t
  // sin(t) = opp / hyp
  // t = sin^-1(sin(t))
}

void Shooter::ManualMove(double speed) {
  if (manualOverride) {
    double currentShooterPivotPosition = m_pivotEncoder.GetPosition();

    double speedFactor = 0.01;
    double targetPositionRotations =
        currentShooterPivotPosition + (speed * speedFactor);

    if (IsTargetInRestrictedRange(targetPositionRotations)) {
      m_pivotPIDController.SetReference(targetPositionRotations,
                                        rev::ControlType::kPosition);
    }
  }
}

bool Shooter::ToggleManualOverride() {
  manualOverride = !manualOverride;
  return manualOverride;
}

void Shooter::StopMotors() {
  m_shooterMotorLeft.Set(0);
  m_shooterMotorRight.Set(0);
}
