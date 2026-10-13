#pragma once
#include <Protocol.h>
#include <ManualJogCore.h>
#include "ArmMap.h"

namespace camera_sort {
constexpr size_t ROUTE_SIZE = 14;
constexpr uint32_t BATCH_LIMIT_MS = 15UL * 60UL * 1000UL;
constexpr uint8_t MAX_TRANSFERS = sorter::kMaxObjects;
constexpr int32_t MIN_SEPARATION10 = 300; // Input filter, not a collision model.
constexpr int32_t TARGET_RADIUS10 = 80;

struct Plan {
  sorter::JointPose poses[ROUTE_SIZE];
  sorter::Detection target;
  uint8_t beforeCounts[sorter::kColorCount];
};

inline bool sideStart(const sorter::Detection& target,
                      const sorter::MotionCalibration& c, int32_t& startY10) {
  if (app::SIDE_APPROACH_MM <= 0 || app::SIDE_APPROACH_MM > 1000 ||
      !sorter::inPickArea(target.x10, target.y10, c)) return false;
  const int64_t y = int64_t(target.y10) - int64_t(app::SIDE_APPROACH_MM)*10;
  if (y < c.pickMinY10 || y > c.pickMaxY10) return false;
  startY10 = int32_t(y);
  return true;
}

inline const char* inspectSideScene(const sorter::Scene& scene,
                                   const sorter::MotionCalibration& c,
                                   uint8_t counts[sorter::kColorCount]) {
  for (uint8_t b = 0; b < sorter::kColorCount; ++b) counts[b] = 0;
  if (scene.count > sorter::kMaxObjects) return "BAD_CAMERA_SCENE";
  for (uint8_t i = 0; i < scene.count; ++i) {
    const sorter::Detection& d = scene.objects[i];
    const uint8_t bin = sorter::classBinIndex(d.classId);
    if (bin >= sorter::kColorCount || !d.pixels) return "BAD_CAMERA_SCENE";
    if (!sorter::inPickArea(d.x10, d.y10, c)) return "OBJECT_OUTSIDE_SORT_AREA";
    ++counts[bin];
  }
  // Bound coordinates before squaring differences.
  for (uint8_t i = 0; i < scene.count; ++i) for (uint8_t j = i + 1; j < scene.count; ++j) {
    const int64_t dx = int64_t(scene.objects[i].x10) - scene.objects[j].x10;
    const int64_t dy = int64_t(scene.objects[i].y10) - scene.objects[j].y10;
    if (dx*dx + dy*dy < int64_t(MIN_SEPARATION10)*MIN_SEPARATION10)
      return "PARTS_TOO_CLOSE";
  }
  for (uint8_t i = 0; i < scene.count; ++i) {
    int32_t startY10;
    if (!sideStart(scene.objects[i], c, startY10)) return "SIDE_APPROACH_OUTSIDE_AREA";
  }
  return nullptr;
}

inline const char* progressError(const sorter::Scene& scene, const Plan& previous,
                                const uint8_t counts[sorter::kColorCount]) {
  for (uint8_t i = 0; i < scene.count; ++i) {
    const sorter::Detection& d = scene.objects[i];
    const int64_t dx = int64_t(d.x10) - previous.target.x10;
    const int64_t dy = int64_t(d.y10) - previous.target.y10;
    if (d.classId == previous.target.classId && dx >= -TARGET_RADIUS10 && dx <= TARGET_RADIUS10 &&
        dy >= -TARGET_RADIUS10 && dy <= TARGET_RADIUS10 &&
        dx*dx + dy*dy <= int64_t(TARGET_RADIUS10)*TARGET_RADIUS10)
      return "TARGET_STILL_PRESENT";
  }
  const uint8_t pickedBin = sorter::classBinIndex(previous.target.classId);
  if (pickedBin >= sorter::kColorCount) return "BAD_CAMERA_SCENE";
  // Each fresh frame must lose exactly one object of the selected colour.
  for (uint8_t b = 0; b < sorter::kColorCount; ++b)
    if (unsigned(counts[b]) + (b == pickedBin ? 1U : 0U) != previous.beforeCounts[b])
      return "SCENE_CHANGED_UNEXPECTEDLY";
  return nullptr;
}

inline bool sidePathClear(const sorter::Detection& target, const sorter::Scene& scene,
                          const sorter::MotionCalibration& c) {
  int32_t startY10;
  if (scene.count > sorter::kMaxObjects || !sideStart(target, c, startY10)) return false;
  for (uint8_t i = 0; i < scene.count; ++i) {
    const sorter::Detection& d = scene.objects[i];
    if (!sorter::inPickArea(d.x10, d.y10, c)) return false;
    if (d.x10 == target.x10 && d.y10 == target.y10 && d.classId == target.classId) continue;
    const int64_t dx = int64_t(d.x10) - target.x10;
    const int64_t dy = d.y10 < startY10 ? int64_t(startY10) - d.y10 :
                       (d.y10 > target.y10 ? int64_t(d.y10) - target.y10 : 0);
    // Centre-distance filter around the nominal segment, not swept-volume sensing.
    if (dx*dx + dy*dy < int64_t(MIN_SEPARATION10)*MIN_SEPARATION10) return false;
  }
  return true;
}

// Lower at (X, Y - 25 mm), enter in +Y with jaws open, grip and lift at target.
// Interpolated joint commands do not imply a straight Cartesian tip trajectory.
inline bool makeSidePlan(const sorter::Detection& target,
                         const uint8_t counts[sorter::kColorCount],
                         const sorter::MotionCalibration& c, const sorter::JogSettings& settings,
                         Plan& result) {
  const uint8_t bin = sorter::classBinIndex(target.classId);
  int32_t startY10;
  sorter::CalibratedPose low = {}, high = {}, sideLow = {}, sideHigh = {};
  if (bin >= sorter::kColorCount || !target.pixels || !sorter::calibrationValid(c, false) ||
      !sideStart(target, c, startY10) ||
      !sorter::interpolateGrid(c, target.x10, target.y10, false, low) ||
      !sorter::interpolateGrid(c, target.x10, target.y10, true, high) ||
      !sorter::interpolateGrid(c, target.x10, startY10, false, sideLow) ||
      !sorter::interpolateGrid(c, target.x10, startY10, true, sideHigh) ||
      !sorter::knownPoseValid(low, c, false) || !sorter::knownPoseValid(high, c, false) ||
      !sorter::knownPoseValid(sideLow, c, false) || !sorter::knownPoseValid(sideHigh, c, false)) return false;
  sorter::JointPose near = low.joints;
  for (uint8_t j = 0; j < 3; ++j)
    near.us[j] = uint16_t((4U*low.joints.us[j] + high.joints.us[j] + 2U)/5U);
  // NEAR is the accepted command interpolation during lift.
  result.poses[0] = c.hubOpen.joints;
  result.poses[1] = sideHigh.joints;
  result.poses[2] = sideLow.joints;
  result.poses[3] = result.poses[4] = low.joints;
  result.poses[5] = near;
  result.poses[6] = high.joints;
  result.poses[7] = c.hubOpen.joints;
  result.poses[8] = result.poses[11] = c.binHighOpen[bin].joints;
  result.poses[9] = result.poses[10] = c.binDropOpen[bin].joints;
  result.poses[12] = c.hubOpen.joints;
  result.poses[13] = c.parkOpen.joints;
  for (size_t i = 0; i < ROUTE_SIZE; ++i) {
    result.poses[i].us[3] = i >= 4 && i <= 9 ? c.closedUs : c.openUs;
    if (!sorter::poseWithinLimits(result.poses[i], c)) return false;
    for (uint8_t j = 0; j < 4; ++j)
      if (result.poses[i].us[j] < settings.minUs[j] || result.poses[i].us[j] > settings.maxUs[j]) return false;
  }
  result.target = target;
  for (uint8_t b = 0; b < sorter::kColorCount; ++b) result.beforeCounts[b] = counts[b];
  return true;
}
} // namespace camera_sort
