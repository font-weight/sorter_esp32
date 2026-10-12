#pragma once
#include <stdint.h>
namespace diagnostic {
constexpr bool OUTPUTS_ENABLED = false;
constexpr bool PINOUT_CONFIRMED = false;
// Example for ESP32-WROOM only. Review the actual board before changing flags.
constexpr uint8_t SERVO_PINS[4] = {25, 26, 27, 33}; // Direct GPIO -> SG90 signal; each signal has a 10 kOhm pull-down to common GND.
// Manual motion-enable switch; no servo-supply voltage monitor is fitted.
constexpr uint8_t ENABLE_PIN = 32;
// Initial narrow BENCH range, not measured mechanical limits.
constexpr uint16_t MIN_US[4] = {1400, 1400, 1400, 1400};
constexpr uint16_t MAX_US[4] = {1600, 1600, 1600, 1600};
constexpr uint16_t MAX_COMMAND_STEP_US = 100;
constexpr uint16_t RATE_US_PER_SECOND = 100;
constexpr uint32_t INACTIVITY_MS = 300000; // 5 minutes between operator commands.
constexpr uint32_t MAX_LOOP_GAP_MS = 250;
inline bool configurationValid() {
  const uint8_t pins[] = {SERVO_PINS[0], SERVO_PINS[1], SERVO_PINS[2], SERVO_PINS[3], ENABLE_PIN};
  for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i)
    for (unsigned j = i + 1; j < sizeof(pins) / sizeof(pins[0]); ++j)
      if (pins[i] == pins[j]) return false;
  for (unsigned j = 0; j < 4; ++j)
    if (MIN_US[j] < 500 || MAX_US[j] > 2500 || MIN_US[j] >= MAX_US[j]) return false;
  return RATE_US_PER_SECOND >= 50 && RATE_US_PER_SECOND <= 250 &&
    MAX_COMMAND_STEP_US > 0 && MAX_COMMAND_STEP_US <= 100 &&
    INACTIVITY_MS >= 1000 && INACTIVITY_MS <= 300000;
}
}
