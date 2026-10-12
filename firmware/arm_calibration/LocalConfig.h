#pragma once
#include <ManualJogCore.h>
namespace manual {
constexpr bool OUTPUTS_ENABLED = true;
constexpr bool PINOUT_CONFIRMED = true;
constexpr bool LIMITS_CONFIRMED = true;
// Direct GPIO -> SG90 signal, with one 10k pulldown to common GND per channel.
// DevKitC V4 / ESP32-WROOM-32U. SG90 signal compatibility was checked separately.
constexpr uint8_t SERVO_PINS[4] = {25, 26, 27, 33};
// A switch connects ENABLE_PIN to 3V3; open switch is denied by INPUT_PULLDOWN.
constexpr uint8_t ENABLE_PIN = 32;
// The owner completed the live UART camera check on this WROOM board.
constexpr bool CAMERA_UART_CONFIRMED = true;
constexpr int CAMERA_RX_PIN = 16, CAMERA_TX_PIN = 17;
constexpr uint32_t CAMERA_BAUD = 115200;
constexpr uint32_t CAMERA_CALIBRATION_ID = 1573755510u;
// User-requested centre-to-centre approach: lower at Y-25 mm, enter in +Y.
// For a 30 mm part, the nominal start is 10 mm before its near face.
// This does not guarantee physical clearance during descent.
constexpr int32_t SIDE_APPROACH_MM = 25;
// Servo supply is not measured. GPIO23/GPIO34 are not used by this sketch.
// The owner confirmed that the corrected linkage is clear at 1500 us.
// Reported individual free ranges: base 500..2500, shoulder 800..2200,
// linkage 600..1700 us. Working bounds keep a 50 us command reserve.
// Gripper: empty jaws first touch at 900 us; open at 2100 us.
// The owner confirmed repeatable grip of the current square part at 1250 us.
// On 2026-10-12, 500 us was confirmed for linkage in the two minY low poses.
// Other part sizes and physical routes still need checking.
// Gripper 950 us is a lower command bound, not the target with a part present.
// These individual checks do not validate combined arm poses or routes.
constexpr uint16_t MIN_US[4] = {550, 850, 500, 950};
constexpr uint16_t MAX_US[4] = {2450, 2150, 1650, 2100};
// A larger command still must fit the individual joint's configured MIN/MAX.
constexpr uint16_t MAX_STEP_US = 100;
constexpr uint16_t RATE_US_PER_SECOND = 300; // User requested 3x the previous 100 us/s.
constexpr uint32_t INACTIVITY_MS = 300000; // 5 minutes between operator commands.
constexpr uint32_t MAX_SESSION_MS = 1800000; // 30 minutes from ARM, not renewable.
inline sorter::JogSettings settings() {
  sorter::JogSettings s = sorter::defaultJogSettings();
  s.outputsEnabled = OUTPUTS_ENABLED;
  s.pinoutConfirmed = PINOUT_CONFIRMED;
  s.limitsConfirmed = LIMITS_CONFIRMED;
  for (uint8_t j = 0; j < 4; ++j) { s.minUs[j] = MIN_US[j]; s.maxUs[j] = MAX_US[j]; }
  s.maxStepUs = MAX_STEP_US;
  s.rateUsPerSecond = RATE_US_PER_SECOND;
  s.holdLeaseMs = INACTIVITY_MS;
  s.maxSessionMs = MAX_SESSION_MS;
  // No seed pose is supplied: enter it explicitly every session.
  return s;
}
inline bool pinsDistinct() {
  const uint8_t p[] = {SERVO_PINS[0], SERVO_PINS[1], SERVO_PINS[2], SERVO_PINS[3], ENABLE_PIN};
  for (uint8_t i = 0; i < 5; ++i) for (uint8_t j = i + 1; j < 5; ++j)
    if (p[i] == p[j]) return false;
  if (CAMERA_UART_CONFIRMED) {
    if (CAMERA_RX_PIN < 0 || CAMERA_TX_PIN < 0 || CAMERA_RX_PIN == CAMERA_TX_PIN) return false;
    for (uint8_t i = 0; i < 5; ++i)
      if (p[i] == CAMERA_RX_PIN || p[i] == CAMERA_TX_PIN) return false;
  }
  return true;
}
} // namespace manual
