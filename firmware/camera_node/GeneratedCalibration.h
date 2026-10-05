#pragma once
#include <stddef.h>
#include <stdint.h>
#include <Homography.h>

// INVALID example. Replace using the PC calibration tool, verify independent
// check points, then explicitly confirm. Pixel identity is NOT millimetres.
namespace sorter_calibration {
constexpr uint32_t CALIBRATION_ID = 0;
constexpr bool CALIBRATION_CONFIRMED = false;
constexpr uint16_t IMAGE_WIDTH = 320;
constexpr uint16_t IMAGE_HEIGHT = 240;
constexpr double HOMOGRAPHY[9] = {1,0,0, 0,1,0, 0,0,1};
constexpr size_t SUPPORT_COUNT = 4;
const sorter::Point2 SUPPORT[SUPPORT_COUNT] = {{8,8},{311,8},{311,231},{8,231}};
}
