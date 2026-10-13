#pragma once
#include <ManualJogCore.h>
#if !defined(SORTER_MOTION_API_VERSION) || SORTER_MOTION_API_VERSION < 4
#error "Update Arduino libraries/SorterCore from this project before compiling sorter_node."
#endif

namespace app {
constexpr uint8_t SERVO_PINS[4] = {25, 26, 27, 33};
constexpr uint8_t ENABLE_PIN = 32; // Switch to 3V3; INPUT_PULLDOWN.
constexpr uint8_t SORT_BUTTON_PIN = 18; // Normally open button to GND; INPUT_PULLUP.
constexpr int CAMERA_RX_PIN = 16, CAMERA_TX_PIN = 17;
constexpr uint32_t CAMERA_BAUD = 115200, USB_BAUD = 115200;
constexpr uint32_t CAMERA_CALIBRATION_ID = 1573755510u;
constexpr uint16_t MIN_US[4] = {550, 850, 500, 950};
constexpr uint16_t MAX_US[4] = {2450, 2150, 1650, 2100};
// CHANGE SPEED HERE ONLY: microseconds of servo pulse per second (50..1000).
constexpr uint16_t RATE_US_PER_SECOND = 800;
static_assert(RATE_US_PER_SECOND >= sorter::kMinJogRateUsPerSecond &&
              RATE_US_PER_SECOND <= sorter::kMaxJogRateUsPerSecond,
              "RATE_US_PER_SECOND must be between 50 and 1000 us/s.");
constexpr int32_t SIDE_APPROACH_MM = 25;
constexpr uint32_t POSE_SAVE_INTERVAL_MS = 500;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;
constexpr uint32_t PARK_SETTLE_MS = 800;
// Change this when servo identities, mounted pulse limits or map conventions change.
constexpr uint32_t POSE_PROFILE_ID = 0x20261013;

inline sorter::JogSettings settings() {
  auto s = sorter::defaultJogSettings();
  s.outputsEnabled = s.pinoutConfirmed = s.limitsConfirmed = true;
  for (uint8_t j=0; j<4; ++j) { s.minUs[j]=MIN_US[j]; s.maxUs[j]=MAX_US[j]; }
  s.rateUsPerSecond = RATE_US_PER_SECOND;
  // The application renews idle holding and re-arms automatically on a new SORT.
  s.holdLeaseMs = 300000;
  s.maxSessionMs = 0; // Production holding has no commissioning session deadline.
  return s;
}
inline bool poseValid(const sorter::JointPose& p) {
  for (uint8_t j=0; j<4; ++j) if (p.us[j]<MIN_US[j] || p.us[j]>MAX_US[j]) return false;
  return true;
}
inline bool pinsDistinct() {
  const int pins[] = {SERVO_PINS[0],SERVO_PINS[1],SERVO_PINS[2],SERVO_PINS[3],
                      ENABLE_PIN,SORT_BUTTON_PIN,CAMERA_RX_PIN,CAMERA_TX_PIN};
  for (unsigned i=0;i<sizeof(pins)/sizeof(pins[0]);++i)
    for (unsigned j=i+1;j<sizeof(pins)/sizeof(pins[0]);++j) if (pins[i]==pins[j]) return false;
  return true;
}
} // namespace app
