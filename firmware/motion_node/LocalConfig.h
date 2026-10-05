#pragma once
#include <MotionCore.h>

namespace local {
// Leave the defaults for a software-only demonstration. Every value below is a
// placeholder; the owner has not yet identified the controller board/mechanics.
constexpr bool VIRTUAL_MODE = true;
constexpr bool MOTOR_OUTPUTS_ENABLED = false;
constexpr bool PINOUT_CONFIRMED = false;
constexpr bool UART_PROFILE_CONFIRMED = false;
// EXAMPLE ONLY: classic ESP32-WROOM development board, not ESP32-CAM or WROVER.
constexpr uint8_t SERVO_PINS[4] = {25, 26, 27, 33}; // Project KiCad -> SN74AHCT125 -> SG90.
// Proposed additional inputs: NOT connected in the current project KiCad.
constexpr uint8_t HARDWARE_ENABLE_PIN = 32;
constexpr uint8_t SERVO_POWER_GOOD_PIN = 23;
constexpr int UART_RX_PIN = 16, UART_TX_PIN = 17;
constexpr uint32_t UART_BAUD = 115200;

inline sorter::MotionSettings makeSettings() {
  sorter::MotionSettings s = sorter::defaultMotionSettings();
  s.virtualMode = VIRTUAL_MODE;
  s.outputsEnabled = MOTOR_OUTPUTS_ENABLED;
  s.pinoutConfirmed = PINOUT_CONFIRMED;
  return s;
}
inline sorter::MotionCalibration makeCalibration() {
  // Replace ALL invented pose, workspace, pulse and height values with measured
  // data before hardware use. Flags stay false until the listed tests pass.
  sorter::MotionCalibration c = sorter::exampleVirtualCalibration();
  c.geometryConfirmed = false;
  c.routesVerified = false;
  c.parkSupported = false;
  return c;
}
inline bool pinsDistinct() {
  const int pins[] = {SERVO_PINS[0], SERVO_PINS[1], SERVO_PINS[2], SERVO_PINS[3],
    HARDWARE_ENABLE_PIN, SERVO_POWER_GOOD_PIN, UART_RX_PIN, UART_TX_PIN};
  for (size_t i = 0; i < sizeof(pins)/sizeof(pins[0]); ++i)
    for (size_t j = i + 1; j < sizeof(pins)/sizeof(pins[0]); ++j)
      if (pins[i] == pins[j]) return false;
  return true;
}
} // namespace local
