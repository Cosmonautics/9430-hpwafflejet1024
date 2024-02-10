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

Elevator::Elevator() {
  ConfigureMotors();
  // Use absolute encoder to set the initial position
  double initialPositionInches = positionTracker.LoadPosition();
  SetInitialPosition(initialPositionInches);
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
  positionTracker.SavePosition(currentPositionInches);
}

void Elevator::MoveToPosition(double positionInches) {
  if (positionInches > kElevatorUpperSoftLimit ||
      positionInches < kElevatorLowerSoftLimit) {
    return;  // Position out of bounds
  }
  double targetPositionRotations = InchesToRotations(positionInches);
  m_pidController.SetReference(targetPositionRotations,
                               rev::ControlType::kPosition);
}

void Elevator::MoveToRelativePosition(double positionInches) {
  // Convert the requested move distance to rotations and then to radians
  double targetPositionRotations =
      InchesToRotations(positionInches + GetCurrentPosition());
  m_pidController.SetReference(targetPositionRotations,
                               rev::ControlType::kPosition);
}

double Elevator::GetCurrentPosition() {
  // Use the relative encoder to get the current position
  double currentPositionRotations =
      m_ElevatorRelativeEncoder.GetPosition() / kElevatorEncoderResolution;
  return RotationsToInches(currentPositionRotations);
}

void Elevator::SetInitialPosition(double positionInches) {
  currentPositionInches = positionInches;
  // Assume this method is called only at initialization or when manually
  // resetting position
  m_ElevatorRelativeEncoder.SetPosition(InchesToRotations(positionInches) *
                                        kElevatorEncoderResolution);
  // Also save this as the new initial position
  positionTracker.SavePosition(positionInches);
}

void Elevator::UpdatePosition() {
  // Update currentPositionInches using the relative encoder
  currentPositionInches = GetCurrentPosition();
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
