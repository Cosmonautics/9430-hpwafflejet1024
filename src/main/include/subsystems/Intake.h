#include "Elevator.h"
#include "Shooter.h"

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  frc2::Command* GetAutonomousCommand();

 private:
  // 2 motors
  // Design:
  // 1 for pivot angle
  rev::CANSparkMax m_intakePivotMax;
  // 1 motor for rollers
  rev::CANSparkFlex m_intakeRollersFlex;

  void PickUpNote(bool isPressed, double speed);

  void DropNote(bool isPressed, double speed);

  void ControlIntakeMotors(bool isPressed, double speed);
  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();

  // 1 throughbore encoder
};