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
  m_pidController.SetFeedbackDevice(m_ElevatorEncoder);
  m_pidController.SetP(kP);
  m_pidController.SetI(kI);
  m_pidController.SetD(kD);
  m_pidController.SetOutputRange(-1.0, 1.0);
}

void Elevator::Periodic() { UpdatePosition(); }

void Elevator::MoveToPosition(double positionInches) {
  if (positionInches > kElevatorUpperSoftLimit ||
      positionInches < kElevatorLowerSoftLimit) {
    return;  // Position out of bounds
  }
  double targetPositionRotations = InchesToRotations(positionInches);
  m_pidController.SetReference(targetPositionRotations,
                               rev::ControlType::kPosition);
}

void Elevator::UpdatePosition() {
  double currentPositionRotations = m_ElevatorEncoder.GetPosition();
  currentPositionInches = RotationsToInches(currentPositionRotations);
}

double Elevator::CalculateTargetHeight(units::degree_t targetRevolutions) {
  units::degree_t currentRevolutions = units::degree_t{
      m_ElevatorEncoder.GetPosition() / kElevatorEncoderResolution};

  units::degree_t revolutionDifference = targetRevolutions - currentRevolutions;

  return revolutionDifference.to<double>();
}

double Elevator::InchesToRotations(double inches) {
  return (((kGearBoxScale) * (inches)) / (kPullyDiameter * M_PI));
}

double Elevator::RotationsToInches(double rotations) {
  return (rotations * kPullyDiameter * M_PI) / kGearBoxScale;
}

bool Elevator::AtTargetPosition() {
  bool flag = std::abs(currentPositionInches - targetPositionInches) <=
              ElevatorConstants::kPositionToleranceInches;
  return flag;
}

bool Elevator::ToggleManualOverride() {
  manualOverride = !manualOverride;
  return manualOverride;
}

void Elevator::ManualMove(double speed) {
  if (manualOverride) {
    double currentElevatorPosition = m_ElevatorEncoder.GetPosition();
    double currentElevatorRotations =
        RotationsToInches(currentElevatorPosition);

    if (currentElevatorRotations > kElevatorLowerSoftLimit &&
        currentElevatorRotations < kElevatorUpperSoftLimit) {
      m_ElevatorMotorLeft.Set(speed * 0.25);
    }
  }
}