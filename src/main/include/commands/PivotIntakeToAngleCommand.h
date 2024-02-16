#ifndef PIVOT_INTAKE_TO_ANGLE_COMMAND_H
#define PIVOT_INTAKE_TO_ANGLE_COMMAND_H

#include <frc2/command/InstantCommand.h>
#include "subsystems/Intake.h"

class PivotIntakeToAngleCommand : public frc2::InstantCommand {
public:
    PivotIntakeToAngleCommand(Intake* intakeSubsystem, double intakeAngleDegrees);
    
    void Initialize() override;
    
private:
    Intake* m_intakeSubsystem;
    double m_intakeAngleDegrees;
};

#endif // PIVOT_INTAKE_TO_ANGLE_COMMAND_H
