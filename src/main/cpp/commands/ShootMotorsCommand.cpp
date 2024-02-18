#include "commands/ShootMotorsCommand.h"

ShootMotorsCommand::ShootMotorsCommand(Shooter& shooter, bool activate, double speed)
    : m_shooter(shooter), m_activate(activate), m_speed(speed) {}

void ShootMotorsCommand::Initialize() {}

void ShootMotorsCommand::Execute() {
    if (m_activate) {
        m_shooter.ShootMotors(true, m_speed);
    } else {
        m_shooter.ShootMotors(false, m_speed);
    }
}

bool ShootMotorsCommand::IsFinished() {
    return false;
}
