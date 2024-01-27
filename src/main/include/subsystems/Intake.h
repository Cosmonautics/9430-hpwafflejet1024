#include <"Elevator.h">
#include <"Shooter.h">

// 2 motors 
    //Design:
        // 1 for pivot angle
  rev::CANSparkMax m_intakePivotMax;
        // 1 motor for rollers
  rev::CANSparkFlex m_intakeRollersFlex;

// 1 throughbore encoder
