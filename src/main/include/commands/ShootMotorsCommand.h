#ifndef SHOOT_MOTORS_COMMAND_H
#define SHOOT_MOTORS_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Shooter.h"

class ShootMotorsCommand : public frc2::CommandHelper<frc2::Command, ShootMotorsCommand> {
public:
    ShootMotorsCommand(Shooter& shooter, bool activate, double speed);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;

private:
    Shooter& m_shooter;
    bool m_activate;
    double m_speed;
};

#endif // SHOOT_MOTORS_COMMAND_H
