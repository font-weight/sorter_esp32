#pragma once
#include <MotionCalibration.h>
#include "Config.h"

namespace arm_map {
constexpr sorter::JointPose PARK = {{1500,1200,1500,2100}};
constexpr sorter::JointPose HUB = {{1500,1500,1500,2100}};

// Measured corner layers, upper center, transit pose and red/blue bins.
// Interior commands are interpolated. The lower center and PARK XYZ have
// no independent measurement; PARK retains its accepted joint commands.
inline sorter::MotionCalibration calibration() {
  sorter::MotionCalibration c = {};
  c.calibrationId = app::CAMERA_CALIBRATION_ID;
  c.pickMinX10 = 200; c.pickMaxX10 = 1300;
  c.pickMinY10 = 250; c.pickMaxY10 = 800;
  c.lowZ10 = 100; c.highZ10 = 1000;
  c.minX10 = -600; c.maxX10 = 2550;
  c.minY10 = -300; c.maxY10 = 800;
  c.minZ10 = 100; c.maxZ10 = 1100;
  c.openUs = 2100; c.closedUs = 1250;
  const sorter::JogSettings settings = app::settings();
  for (uint8_t j = 0; j < 4; ++j) {
    c.pulseMin[j] = settings.minUs[j];
    c.pulseMax[j] = settings.maxUs[j];
    c.maxRateUsPerSecond[j] = app::RATE_US_PER_SECOND;
    c.maxAccelerationUsPerSecond2[j] = 100;
  }
  c.grid[0].lowOpen = {{1700,2000,500,2100}};
  c.grid[1].lowOpen = {{1150,2000,500,2100}};
  c.grid[2].lowOpen = {{1600,2150,850,2100}};
  c.grid[3].lowOpen = {{1250,2150,850,2100}};
  c.grid[0].highOpen = {{1700,1350,1250,2100}};
  c.grid[1].highOpen = {{1200,1350,1250,2100}};
  c.grid[2].highOpen = {{1600,1700,1350,2100}};
  c.grid[3].highOpen = {{1250,1700,1350,2100}};
  for (uint8_t i = 0; i < 4; ++i) {
    c.grid[i].measuredLow = true;
    c.grid[i].measuredHigh = true;
  }
  c.centerHighEnabled = true;
  c.centerHighOpen = {{{1441,1426,1220,2100}}, {750,525,990}, true};
  c.hubOpen = {HUB, {550,550,1100}, true};
  c.parkOpen = {PARK, c.hubOpen.tip, false};
  c.binHighOpen[0] = {{{2100,1600,850,2100}}, {-600,-250,600}, true};
  c.binDropOpen[0] = {{{2100,1800,750,2100}}, {-600,-250,400}, true};
  c.binHighOpen[1] = {{{800,1600,850,2100}}, {2550,-300,600}, true};
  c.binDropOpen[1] = {{{800,1800,750,2100}}, {2550,-300,400}, true};
  return c;
}
} // namespace arm_map
