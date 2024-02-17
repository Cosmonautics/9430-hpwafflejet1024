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

Elevator::Elevator() { ConfigureMotors(); }

void Elevator::ConfigureMotors() {
  m_ElevatorMotorRight.Follow(m_ElevatorMotorLeft, true);

  // Configure PID controller on SparkMax
  m_ElevatorPIDController.SetFeedbackDevice(m_ElevatorThroughBoreEncoder);
  m_ElevatorPIDController.SetP(kP);
  m_ElevatorPIDController.SetI(kI);
  m_ElevatorPIDController.SetD(kD);
  m_ElevatorPIDController.SetOutputRange(-1.0, 1.0);
}

void Elevator::Periodic() { UpdatePosition(); }

void Elevator::MoveToPosition(double positionInches) {
  if (positionInches > kElevatorUpperSoftLimit ||
      positionInches < kElevatorLowerSoftLimit) {
    return;  // Position out of bounds
  }
  double targetPositionRotations = CalculateRotations(positionInches); // change this to my new function 
  m_ElevatorPIDController.SetReference(targetPositionRotations,
                               rev::ControlType::kPosition);
}

void Elevator::UpdatePosition() {
  double currentPositionRotations = m_ElevatorThroughBoreEncoder.GetPosition();
  currentPositionInches = RotationsToInches(currentPositionRotations);
}

double Elevator::CalculateTargetHeight(units::degree_t targetRevolutions) {
  units::degree_t currentRevolutions = units::degree_t{
      m_ElevatorThroughBoreEncoder.GetPosition() / kElevatorEncoderResolution};

  units::degree_t revolutionDifference = targetRevolutions - currentRevolutions;

  return revolutionDifference.to<double>();
}

double Elevator::CalculateRotations(double height) {
  return (((kGearBoxScale) * (height)) / (kPullyDiameter * M_PI));
}

double Elevator::RotationsToInches(double rotations) {
  return (rotations * kPullyDiameter * M_PI) / kGearBoxScale;
}

bool Elevator::AtTargetPosition() {
  bool flag = std::abs(currentPositionInches - targetPositionInches) <=
              ElevatorConstants::kPositionToleranceInches;
  return flag;
}
