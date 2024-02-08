// include statements

// if X button pressed, move X meters

// set according to power / xbox button pressed

// bunch of static constant expressions defining basic physic limits and
// voltages

// setting variables for joystick, encoder, and sparkmax from FRC module

// create PID controller with constraints

#include "subsystems/Elevator.h"

#include "Constants.h"

using namespace ElevatorConstants;

Elevator::Elevator() { ConfigureMotors(); }

void Elevator::Periodic() { UpdatePosition(); }

void Elevator::MoveToPosition(double positionInches) {
  // TODO:
  // Add logic to keep track of absolute elevator position
  MoveToRelativePosition(positionInches);
}

void Elevator::MoveToRelativePosition(double positionInches) {
  double currentPositionInchesLocal =
      ConvertEncoderUnitsToInches(m_ElevatorEncoder.GetPosition());

  double newPositionInches = currentPositionInchesLocal + positionInches;

  if (newPositionInches > kElevatorUpperSoftLimit ||
      newPositionInches < kElevatorLowerSoftLimit) {
    return;
  }

  double targetPositionUnits = ConvertInchesToEncoderUnits(newPositionInches);

  m_pidController.SetReference(targetPositionUnits,
                               rev::ControlType::kPosition);

  targetPositionInches = newPositionInches;
}

double Elevator::ConvertInchesToEncoderUnits(double inches) {
  double circumference = M_PI * kPullyDiameter;
  double encoderUnitsPerInch = kElevatorEncoderResolution / circumference;
  return static_cast<int>(inches * encoderUnitsPerInch);
}

double Elevator::ConvertEncoderUnitsToInches(double units) {
  double circumference = M_PI * kPullyDiameter;
  double inchesPerEncoderUnit = circumference / kElevatorEncoderResolution;
  return units * inchesPerEncoderUnit;
}

bool Elevator::AtTargetPosition() {
  bool flag = std::abs(currentPositionInches - targetPositionInches) <=
              ElevatorConstants::kPositionToleranceInches;
  return flag;
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

void Elevator::UpdatePosition() {
  currentPositionInches =
      ConvertEncoderUnitsToInches(m_ElevatorEncoder.GetPosition());
}

void Elevator::Move(double speed) {
  m_ElevatorMotorLeft.Set(speed);
  // m_ElevatorMotorRight is following m_ElevatorMotorLeft in inverted mode, so
  // no need to set it separately
}

double GetCurrentPosition() {
    return m_ElevatorEncoder.GetPosition();
}
double CalculateTargetHeight(units::angle::degrees theta2) {
    units::angle::degrees theta1 = m_ElevatorEncoder.GetPosition();
    ElevatorConstants::kElevatorDrumDiameterInches * (theta2 - )
}

