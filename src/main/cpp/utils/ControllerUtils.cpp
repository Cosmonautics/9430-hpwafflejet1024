#include "utils/ControllerUtils.h"

void ControllerUtils::VibrateController(frc::XboxController& controller,
                                        double intensity,
                                        units::second_t duration) {
  controller.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, intensity);
  controller.SetRumble(frc::GenericHID::RumbleType::kRightRumble, intensity);

  frc::Timer timer;
  timer.Start();
  while (timer.Get() < duration) {}

  controller.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0);
  controller.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0);
}
