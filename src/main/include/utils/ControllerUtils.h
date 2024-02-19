#pragma once

#include <frc/XboxController.h>
#include <frc/Timer.h>
#include <units/time.h>

class ControllerUtils {
public:
    static void VibrateController(frc::XboxController& controller, double intensity, units::second_t duration);
};
