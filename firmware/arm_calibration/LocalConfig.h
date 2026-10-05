#pragma once
#include <ManualJogCore.h>
namespace manual {
constexpr bool OUTPUTS_ENABLED = false;
constexpr bool PINOUT_CONFIRMED = false;
constexpr bool LIMITS_CONFIRMED = false;
// Existing project PWM nets. Confirm the physical controller and assembled arm.
constexpr uint8_t SERVO_PINS[4] = {25, 26, 27, 33};
// PROPOSED wiring additions. These inputs are absent from the current schematic.
constexpr uint8_t ENABLE_PIN = 32, POWER_GOOD_PIN = 23;
inline sorter::JogSettings settings() {
  sorter::JogSettings s = sorter::defaultJogSettings();
  s.outputsEnabled = OUTPUTS_ENABLED;
  s.pinoutConfirmed = PINOUT_CONFIRMED;
  s.limitsConfirmed = LIMITS_CONFIRMED;
  // Replace 1400..1600 with the initially established local pulse limits for
  // each mounted joint; extend a limit only after physically checking clearance.
  // No seed pose is supplied: the operator must enter it explicitly every session.
  return s;
}
inline bool pinsDistinct() {
  const uint8_t p[] = {SERVO_PINS[0], SERVO_PINS[1], SERVO_PINS[2], SERVO_PINS[3], ENABLE_PIN, POWER_GOOD_PIN};
  for (uint8_t i = 0; i < 6; ++i) for (uint8_t j = i + 1; j < 6; ++j)
    if (p[i] == p[j]) return false;
  return true;
}
} // namespace manual
