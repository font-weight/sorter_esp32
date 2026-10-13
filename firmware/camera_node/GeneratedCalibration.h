#pragma once
#include <stddef.h>
#include <stdint.h>
#include <Homography.h>

// Measured camera-to-workspace calibration used by the assembled sorter.
// Preserve these values together with the fixed sensor settings in CameraConfig.h.
namespace sorter_calibration {
constexpr uint32_t CALIBRATION_ID = 1573755510u;
constexpr bool CALIBRATION_CONFIRMED = true;
constexpr uint16_t IMAGE_WIDTH = 320;
constexpr uint16_t IMAGE_HEIGHT = 240;
constexpr double HOMOGRAPHY[9] = {-0.44368931570979614, -0.01774234808352327, 153.59603638292995, 0.0044056401622834824, 0.41575937433079874, 9.392491442807172, 0.00015980676919558911, -0.00012286104300969681, 1};
constexpr size_t SUPPORT_COUNT = 8;
const sorter::Point2 SUPPORT[SUPPORT_COUNT] = {
    {6.9000000000000004, 222.80000000000001},
    {7.7999999999999998, 119.8},
    {8.8000000000000007, 13.9},
    {159.80000000000001, 11.4},
    {310.39999999999998, 11.4},
    {306.60000000000002, 125.3},
    {301.89999999999998, 230.5},
    {158.90000000000001, 228},
};
}  // namespace sorter_calibration
