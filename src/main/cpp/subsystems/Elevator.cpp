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
  if (positionInches > kElevatorUpperSoftLimit ||
      positionInches < kElevatorLowerSoftLimit) {
    return;
  }
  targetPositionInches = positionInches;
  double targetPositionUnits = ConvertInchesToEncoderUnits(positionInches);
  // Instead of using m_pidController.Calculate, directly set the target for
  // SparkMax PID
  m_ElevatorMotorLeft.GetPIDController().SetReference(
      targetPositionUnits, rev::ControlType::kPosition);
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

bool Elevator::AtTargetPosition() const {
  return std::abs(currentPositionInches - targetPositionInches) <=
         ElevatorConstants::kPositionToleranceInches;
}

void Elevator::ConfigureMotors() {
  m_ElevatorMotorLeft.RestoreFactoryDefaults();
  m_ElevatorMotorRight.RestoreFactoryDefaults();
  m_ElevatorMotorRight.Follow(m_ElevatorMotorLeft, true);

  // Configure PID controller on SparkMax
  auto pidController = m_ElevatorMotorLeft.GetPIDController();
  pidController.SetP(kP);
  pidController.SetI(kI);
  pidController.SetD(kD);
  pidController.SetOutputRange(-1.0, 1.0);
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
