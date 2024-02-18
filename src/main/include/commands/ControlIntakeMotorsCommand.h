#ifndef CONTROL_INTAKE_MOTORS_COMMAND_H
#define CONTROL_INTAKE_MOTORS_COMMAND_H

#include <frc2/command/InstantCommand.h>
#include "subsystems/Intake.h"

class ControlIntakeMotorsCommand : public frc2::InstantCommand {
public:
    ControlIntakeMotorsCommand(Intake* intakeSubsystem, bool isPressed, double speed);
    
    void Initialize() override;
    
private:
    Intake* m_intakeSubsystem;
    bool m_isPressed;
    double m_speed;
};

#endif // CONTROL_INTAKE_MOTORS_COMMAND_H
