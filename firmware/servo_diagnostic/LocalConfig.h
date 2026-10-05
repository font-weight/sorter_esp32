#pragma once
#include <stdint.h>
namespace diagnostic {
constexpr bool OUTPUTS_ENABLED = false;
constexpr bool PINOUT_CONFIRMED = false;
// Example for ESP32-WROOM only. Review the actual board before changing flags.
constexpr uint8_t SERVO_PINS[4] = {25, 26, 27, 33}; // KiCad -> SN74AHCT125 -> SG90.
// Proposed additional inputs, absent from current KiCad; hardware flags stay false.
constexpr uint8_t ENABLE_PIN = 32, POWER_GOOD_PIN = 23;
// Initial narrow BENCH range, not measured mechanical limits.
constexpr uint16_t MIN_US[4] = {1400, 1400, 1400, 1400};
constexpr uint16_t MAX_US[4] = {1600, 1600, 1600, 1600};
constexpr uint16_t MAX_COMMAND_STEP_US = 25;
constexpr uint16_t RATE_US_PER_SECOND = 100;
constexpr uint32_t INACTIVITY_MS = 10000;
constexpr uint32_t MAX_LOOP_GAP_MS = 250;
inline bool configurationValid() {
  const uint8_t pins[] = {SERVO_PINS[0], SERVO_PINS[1], SERVO_PINS[2], SERVO_PINS[3], ENABLE_PIN, POWER_GOOD_PIN};
  for (unsigned i = 0; i < 6; ++i) for (unsigned j = i + 1; j < 6; ++j)
    if (pins[i] == pins[j]) return false;
  for (unsigned j = 0; j < 4; ++j)
    if (MIN_US[j] < 500 || MAX_US[j] > 2500 || MIN_US[j] >= MAX_US[j]) return false;
  return RATE_US_PER_SECOND >= 50 && RATE_US_PER_SECOND <= 250 &&
    MAX_COMMAND_STEP_US > 0 && MAX_COMMAND_STEP_US <= 25;
}
}
