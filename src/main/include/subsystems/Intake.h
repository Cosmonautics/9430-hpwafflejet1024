#include <"Elevator.h">
#include <"Shooter.h">

// 2 motors 
    //Design:
        // 1 for pivot angle
  rev::CANSparkMax m_intakePivotMax;
        // 1 motor for rollers
  rev::CANSparkFlex m_intakeRollersFlex;

  void PickUpNote(bool isPressed, double speed);

  void DropNote(bool isPressed, double speed);

  void ControlIntakeMotors(bool isPressed, double speed);

// 1 throughbore encoder
