// include statements

// if X button pressed, move X meters

// set according to power / xbox button pressed

// bunch of static constant expressions defining basic physic limits and
// voltages

// setting variables for joystick, encoder, and sparkmax from FRC module

// create PID controller with constraints

#include "subsystems/Elevator.h"

#include <cmath>  // For std::abs

#include "Constants.h"

using namespace ElevatorConstants;

Elevator::Elevator() : m_TotalRotations(0.0), m_LastEncoderPosition(0.0) {
  ConfigureMotors();
  // Use absolute encoder to set the initial position
  double initialPositionInches = positionTracker.LoadPosition();
  SetInitialPosition(initialPositionInches);
  m_LastEncoderPosition =
      m_ElevatorEncoder.GetPosition();  // Initialize last encoder position
}

void Elevator::ConfigureMotors() {
  m_ElevatorMotorLeft.RestoreFactoryDefaults();
  m_ElevatorMotorRight.RestoreFactoryDefaults();
  m_ElevatorMotorRight.Follow(m_ElevatorMotorLeft, true);

  // Configure PID controller on SparkMax
  m_pidController.SetP(kP);
  m_pidController.SetI(kI);
  m_pidController.SetD(kD);
  m_pidController.SetOutputRange(-1.0, 1.0);
}

void Elevator::Periodic() {
  UpdatePosition();
  CheckForFullRotation();
}

void Elevator::MoveToPosition(double positionInches) {
  if (positionInches > kElevatorUpperSoftLimit ||
      positionInches < kElevatorLowerSoftLimit) {
    return;  // Position out of bounds
  }
  double targetPositionRotations = InchesToRotations(positionInches);
  m_pidController.SetReference(
      targetPositionRotations + m_TotalRotations * kElevatorEncoderResolution,
      rev::ControlType::kPosition);
}

void Elevator::MoveToRelativePosition(double positionInches) {
  double targetPositionInches = positionInches + GetCurrentPosition();
  if (targetPositionInches > kElevatorUpperSoftLimit ||
      targetPositionInches < kElevatorLowerSoftLimit) {
    return;  // Position out of bounds
  }
  double targetPositionRotations = InchesToRotations(targetPositionInches);
  m_pidController.SetReference(
      targetPositionRotations + m_TotalRotations * kElevatorEncoderResolution,
      rev::ControlType::kPosition);
}

void Elevator::CheckForFullRotation() {
  double currentEncoderPosition = m_ElevatorEncoder.GetPosition();
  double rotationsSinceLastCheck =
      currentEncoderPosition - m_LastEncoderPosition;

  // Check for a full rotation (positive or negative)
  if (std::abs(rotationsSinceLastCheck) >= kElevatorEncoderResolution) {
    // Calculate how many full rotations were made since the last check
    int fullRotations =
        static_cast<int>(rotationsSinceLastCheck / kElevatorEncoderResolution);
    m_TotalRotations += fullRotations;

    m_LastEncoderPosition += fullRotations * kElevatorEncoderResolution;
  }
}

double Elevator::GetCurrentPosition() {
  double currentPositionRotations = m_ElevatorEncoder.GetPosition();
  return RotationsToInches(currentPositionRotations);
}

void Elevator::SetInitialPosition(double positionInches) {
  currentPositionInches = positionInches;
  m_ElevatorRelativeEncoder.SetPosition(0);  // Reset encoder to 0
  m_TotalRotations = InchesToRotations(positionInches) /
                     kElevatorEncoderResolution;  // Set total rotations based
                                                  // on initial position
  m_LastEncoderPosition = 0.0;                    // Reset last encoder position
  positionTracker.SavePosition(positionInches);
}

void Elevator::UpdatePosition() {
  double currentPositionRotations = m_ElevatorEncoder.GetPosition();
  currentPositionInches =
      RotationsToInches(currentPositionRotations + m_TotalRotations);
  positionTracker.SavePosition(currentPositionInches);
}

double Elevator::CalculateTargetHeight(units::degree_t targetRevolutions) {
  units::degree_t currentRevolutions = units::degree_t{
      m_ElevatorEncoder.GetPosition() / kElevatorEncoderResolution};

  units::degree_t revolutionDifference = targetRevolutions - currentRevolutions;

  return revolutionDifference.to<double>();
}

double Elevator::InchesToRotations(double inches) {
  return inches / (kPullyDiameter * M_PI);
}

double Elevator::RotationsToInches(double rotations) {
  return rotations * (kPullyDiameter * M_PI);
}

bool Elevator::AtTargetPosition() {
  bool flag = std::abs(currentPositionInches - targetPositionInches) <=
              ElevatorConstants::kPositionToleranceInches;
  return flag;
}
