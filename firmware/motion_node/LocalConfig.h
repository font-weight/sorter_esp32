#pragma once
#include <MotionCore.h>

namespace local {
// Measured MeArm pick/bin poses and the owner's new camera PARK command.
// Physical interpolation, loaded routes and park support still need verification.
constexpr bool VIRTUAL_MODE = true;
constexpr bool MOTOR_OUTPUTS_ENABLED = false;
constexpr bool PINOUT_CONFIRMED = false;
// Live-camera virtual cycles use UART while physical PWM stays disabled.
constexpr bool UART_PROFILE_CONFIRMED = true;
// Leave time for both slow virtual routes and manual removal from the camera view.
constexpr uint32_t VIRTUAL_BATCH_LIMIT_MS = 15UL * 60UL * 1000UL;
// Confirmed manual-test board: ESP32-WROOM-32U DevKitC V4.
constexpr uint8_t SERVO_PINS[4] = {25, 26, 27, 33}; // Direct GPIO -> SG90 signal; each signal has a 10 kOhm pull-down to common GND.
// Manual motion-enable switch; no servo-supply voltage monitor is fitted.
constexpr uint8_t HARDWARE_ENABLE_PIN = 32;
constexpr int UART_RX_PIN = 16, UART_TX_PIN = 17;
constexpr uint32_t UART_BAUD = 115200;

inline sorter::MotionSettings makeSettings() {
  sorter::MotionSettings s = sorter::defaultMotionSettings();
  s.virtualMode = VIRTUAL_MODE;
  s.outputsEnabled = MOTOR_OUTPUTS_ENABLED;
  s.pinoutConfirmed = PINOUT_CONFIRMED;
  if (VIRTUAL_MODE) s.maxBatchMs = VIRTUAL_BATCH_LIMIT_MS;
  return s;
}
inline sorter::MotionCalibration makeCalibration() {
  sorter::MotionCalibration c = {};
  // Must match the measured camera calibration in GeneratedCalibration.h.
  c.calibrationId = 1573755510u;
  c.geometryConfirmed = false;
  c.routesVerified = false;
  c.parkSupported = false;

  // Coordinates use 0.1 mm; pulses use microseconds. Y=80 was confirmed as a
  // correction of the two left maxY CSV entries by the owner on 2026-10-12.
  c.pickMinX10 = 200; c.pickMaxX10 = 1300;
  c.pickMinY10 = 250; c.pickMaxY10 = 800;
  c.lowZ10 = 100; c.highZ10 = 1000;
  // Corner order: (minX,minY), (maxX,minY), (minX,maxY), (maxX,maxY).
  c.grid[0] = {{{1700,2000,500,2100}}, {{1700,1350,1250,2100}}, true, true};
  c.grid[1] = {{{1150,2000,500,2100}}, {{1200,1350,1250,2100}}, true, true};
  c.grid[2] = {{{1600,2150,850,2100}}, {{1600,1700,1350,2100}}, true, true};
  c.grid[3] = {{{1250,2150,850,2100}}, {{1250,1700,1350,2100}}, true, true};

  // Actual upper center confirmed by the owner; preserve its measured 99 mm.
  // The lower center is still missing. A partly refined map stays virtual.
  c.centerHighEnabled = true;
  c.centerHighOpen = {{{1441,1426,1220,2100}}, {750,525,990}, true};
  c.centerLowEnabled = false;

  // Envelope of measured endpoints, not proof of the physical swept path.
  c.minX10 = -600; c.maxX10 = 2550;
  c.minY10 = -300; c.maxY10 = 800;
  c.minZ10 = 100; c.maxZ10 = 1100;
  const uint16_t pulseMin[4] = {550,850,500,950};
  const uint16_t pulseMax[4] = {2450,2150,1650,2100};
  for (uint8_t j = 0; j < 4; ++j) {
    c.pulseMin[j] = pulseMin[j]; c.pulseMax[j] = pulseMax[j];
    // Initial command limits for commissioning, not measured joint capability.
    c.maxRateUsPerSecond[j] = 100;
    c.maxAccelerationUsPerSecond2[j] = 100;
  }
  // 500 us was checked in the two minY low poses; it is not a general proof
  // that the linkage can move freely to 500 us in every arm configuration.
  c.openUs = 2100; c.closedUs = 1250;
  c.hubOpen = {{{1500,1500,1500,2100}}, {550,550,1100}, true};
  // Actual XYZ of the new PARK have not been measured. Reuse HUB's point only
  // as a numeric virtual placeholder; measured=false prevents physical approval.
  c.parkOpen = {{{1500,1200,1500,2100}}, c.hubOpen.tip, false};
  // Red class 1 -> bin 0; blue class 3 -> bin 1.
  c.binHighOpen[0] = {{{2100,1600,850,2100}}, {-600,-250,600}, true};
  c.binDropOpen[0] = {{{2100,1800,750,2100}}, {-600,-250,400}, true};
  c.binHighOpen[1] = {{{800,1600,850,2100}}, {2550,-300,600}, true};
  c.binDropOpen[1] = {{{800,1800,750,2100}}, {2550,-300,400}, true};
  return c;
}
inline bool pinsDistinct() {
  const int pins[] = {SERVO_PINS[0], SERVO_PINS[1], SERVO_PINS[2], SERVO_PINS[3],
    HARDWARE_ENABLE_PIN, UART_RX_PIN, UART_TX_PIN};
  for (size_t i = 0; i < sizeof(pins)/sizeof(pins[0]); ++i)
    for (size_t j = i + 1; j < sizeof(pins)/sizeof(pins[0]); ++j)
      if (pins[i] == pins[j]) return false;
  return true;
}
} // namespace local
