#pragma once
#include <ColorDetector.h>

// Confirm each profile only after the corresponding physical check.
constexpr bool CAMERA_PROFILE_CONFIRMED = true;
constexpr bool CAMERA_UART_PINS_CONFIRMED = false;
constexpr bool VISION_CONFIG_CONFIRMED = false;
constexpr unsigned USB_BAUD = 460800;
constexpr unsigned CAMERA_UART_BAUD = 115200;
constexpr int CAMERA_UART_RX = 14; // Project KiCad: controller TX17 -> camera RX14; no SD.
constexpr int CAMERA_UART_TX = 13; // Project KiCad: camera TX13 -> controller RX16.

// Fixed OV2640 settings: starting values, not measured values. CAPTURE and
// recognition use the SAME settings. Recalibrate after changing these.
constexpr int SENSOR_EXPOSURE = 80; // 0..1200
constexpr int SENSOR_GAIN = 0;       // 0..30
constexpr int SENSOR_WB_MODE = 1;    // 1 sunny, 2 cloudy, 3 office, 4 home
constexpr bool SENSOR_HMIRROR = false;
constexpr bool SENSOR_VFLIP = false;

// Red and blue only. Keep thresholds tuned from the actual parts and lighting.
// Class 2 (green) is disabled; overlapping ranges are rejected as unknown.
const sorter::ColorRange COLOR_RANGES[] = {
  {sorter::kRedClassId, 250, 255, 0, 200, 0, 239, 60}, // red -> first bin
  {sorter::kBlueClassId, 100, 189, 100, 190, 190, 255, 50} // blue -> second bin
};
static_assert(sizeof(COLOR_RANGES)/sizeof(COLOR_RANGES[0]) == sorter::kColorCount,
              "The sorter must have exactly two color ranges");
const sorter::DetectorConfig DETECTOR_CONFIG = {
  COLOR_RANGES, sizeof(COLOR_RANGES)/sizeof(COLOR_RANGES[0]),
  {8, 8, 312, 232}, // half-open ROI in 320x240 frame; set to your tray
  40, 6000, true   // pixel area range and ROI-edge rejection
};
