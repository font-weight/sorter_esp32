#pragma once
#include <Arduino.h>
#include <ColorDetector.h>

// All are intentionally false: actual board, wiring, lighting and parts are
// unavailable. Change only after the corresponding physical check.
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

// Teaching examples only. Adjust from recorded images of actual parts and
// background under the final illumination; overlap is rejected as unknown.
const sorter::ColorRange COLOR_RANGES[] = {
  {1, 250, 255, 0, 200, 0, 239, 60}, // red
  {2, 0, 206, 140, 255, 0, 222, 45}, // green
  {3, 100, 189, 100, 190, 190, 255, 50}  // blue
};
const sorter::DetectorConfig DETECTOR_CONFIG = {
  COLOR_RANGES, sizeof(COLOR_RANGES)/sizeof(COLOR_RANGES[0]),
  {8, 8, 312, 232}, // half-open ROI in 320x240 frame; set to your tray
  40, 6000, true   // pixel area range and ROI-edge rejection
};
