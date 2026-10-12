#pragma once
#include <stdint.h>

namespace bench {
// Only for ONE unloaded, disconnected SG90 on the confirmed DevKitC V4.
// Enable after checking the external supply, ground and signal wiring.
constexpr bool OUTPUTS_ENABLED = true;
// Direct signal to SG90; one 10k resistor from this signal to common GND.
constexpr uint8_t SERVO_PIN = 25;
constexpr uint16_t INITIAL_US = 1500;
constexpr uint16_t MIN_US = 1400, MAX_US = 1600;
constexpr uint16_t MAX_STEP_US = 100;
constexpr uint16_t RATE_US_PER_SECOND = 100;
constexpr uint32_t CONFIRM_VALID_MS = 300000; // 5 minutes to enter ARM after CONFIRM.
constexpr uint32_t INACTIVITY_MS = 300000; // 5 minutes between commands.
constexpr uint32_t MAX_SESSION_MS = 1800000; // 30 minutes from ARM.
constexpr uint32_t MAX_LOOP_GAP_MS = 250;
} // namespace bench
