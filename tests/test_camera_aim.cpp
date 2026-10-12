#include "../firmware/arm_calibration/CameraAimConfig.h"
#include "../firmware/motion_node/LocalConfig.h"
#include <assert.h>
#include <stdio.h>

using namespace sorter;

static void samePose(const JointPose& a, const JointPose& b) {
  for (size_t j = 0; j < 4; ++j) assert(a.us[j] == b.us[j]);
}
static void samePoint(const ToolPoint& a, const ToolPoint& b) {
  assert(a.x10 == b.x10 && a.y10 == b.y10 && a.z10 == b.z10);
}
static void sameMeasuredPose(const CalibratedPose& a, const CalibratedPose& b) {
  samePose(a.joints, b.joints);
  samePoint(a.tip, b.tip);
  assert(a.measured == b.measured);
}

int main() {
  const MotionCalibration aim = camera_aim::upperCalibration();
  const MotionCalibration main = local::makeCalibration();
  assert(aim.calibrationId == main.calibrationId);
  assert(aim.calibrationId == manual::CAMERA_CALIBRATION_ID);
  assert(aim.pickMinX10 == main.pickMinX10 && aim.pickMaxX10 == main.pickMaxX10);
  assert(aim.pickMinY10 == main.pickMinY10 && aim.pickMaxY10 == main.pickMaxY10);
  assert(aim.highZ10 == main.highZ10 && aim.lowZ10 == main.lowZ10);
  assert(aim.centerHighEnabled == main.centerHighEnabled);
  assert(aim.openUs == main.openUs);
  sameMeasuredPose(aim.centerHighOpen, main.centerHighOpen);
  sameMeasuredPose(aim.hubOpen, main.hubOpen);
  for (size_t j = 0; j < 4; ++j) {
    assert(aim.pulseMin[j] == main.pulseMin[j]);
    assert(aim.pulseMax[j] == main.pulseMax[j]);
  }
  for (size_t i = 0; i < 4; ++i) {
    samePose(aim.grid[i].highOpen, main.grid[i].highOpen);
    assert(aim.grid[i].measuredHigh == main.grid[i].measuredHigh);
  }
  assert(centerLevelValid(aim, true));
  assert(knownPoseValid(aim.hubOpen, aim, true));

  // The batch map uses the existing measured lower corners and both bins.
  // Its missing lower center and unmeasured PARK must remain explicit.
  const MotionCalibration batch = camera_aim::sortCalibration();
  assert(batch.calibrationId == main.calibrationId);
  assert(batch.geometryConfirmed == main.geometryConfirmed);
  assert(batch.routesVerified == main.routesVerified);
  assert(batch.parkSupported == main.parkSupported);
  assert(batch.centerLowEnabled == main.centerLowEnabled);
  assert(batch.centerHighEnabled == main.centerHighEnabled);
  assert(!batch.centerLowEnabled && batch.centerHighEnabled);
  assert(batch.pickMinX10 == main.pickMinX10 && batch.pickMaxX10 == main.pickMaxX10);
  assert(batch.pickMinY10 == main.pickMinY10 && batch.pickMaxY10 == main.pickMaxY10);
  assert(batch.lowZ10 == main.lowZ10 && batch.highZ10 == main.highZ10);
  assert(batch.minX10 == main.minX10 && batch.maxX10 == main.maxX10);
  assert(batch.minY10 == main.minY10 && batch.maxY10 == main.maxY10);
  assert(batch.minZ10 == main.minZ10 && batch.maxZ10 == main.maxZ10);
  assert(batch.openUs == main.openUs && batch.closedUs == main.closedUs);
  sameMeasuredPose(batch.centerLowOpen, main.centerLowOpen);
  sameMeasuredPose(batch.centerHighOpen, main.centerHighOpen);
  sameMeasuredPose(batch.hubOpen, main.hubOpen);
  sameMeasuredPose(batch.parkOpen, main.parkOpen);
  assert(!batch.parkOpen.measured);
  for (size_t j = 0; j < 4; ++j) {
    assert(batch.pulseMin[j] == main.pulseMin[j]);
    assert(batch.pulseMax[j] == main.pulseMax[j]);
    assert(batch.maxRateUsPerSecond[j] == main.maxRateUsPerSecond[j]);
    assert(batch.maxAccelerationUsPerSecond2[j] == main.maxAccelerationUsPerSecond2[j]);
  }
  for (size_t i = 0; i < 4; ++i) {
    samePose(batch.grid[i].lowOpen, main.grid[i].lowOpen);
    samePose(batch.grid[i].highOpen, main.grid[i].highOpen);
    assert(batch.grid[i].measuredLow == main.grid[i].measuredLow);
    assert(batch.grid[i].measuredHigh == main.grid[i].measuredHigh);
  }
  for (size_t i = 0; i < kColorCount; ++i) {
    sameMeasuredPose(batch.binHighOpen[i], main.binHighOpen[i]);
    sameMeasuredPose(batch.binDropOpen[i], main.binDropOpen[i]);
  }
  assert(calibrationValid(batch, false));
  assert(!calibrationValid(batch, true));

  // Compare results across corners, edges, triangle seams and the measured center.
  const int32_t xs[] = {200,400,750,1000,1300};
  const int32_t ys[] = {250,400,525,650,800};
  for (int32_t x : xs) for (int32_t y : ys) {
    CalibratedPose a = {}, b = {};
    assert(interpolateGrid(aim, x, y, true, a));
    assert(interpolateGrid(main, x, y, true, b));
    samePose(a.joints, b.joints);
    samePoint(a.tip, b.tip);
    assert(a.joints.us[3] == 2100);
  }

  CalibratedPose result = {};
  const JointPose expected = {{1551,1569,1294,2100}};
  assert(interpolateGrid(aim, 400, 650, true, result));
  samePose(result.joints, expected);
  assert(result.tip.x10 == 400 && result.tip.y10 == 650 && result.tip.z10 == 996);
  assert(!result.measured);

  const int32_t outside[][2] = {{199,525},{1301,525},{750,249},{750,801}};
  for (const auto& point : outside)
    assert(!interpolateGrid(aim, point[0], point[1], true, result));

  // A missing measured vertex or a tightened command limit must deny interpolation.
  MotionCalibration invalid = aim;
  invalid.grid[0].measuredHigh = false;
  assert(!interpolateGrid(invalid, 400, 650, true, result));
  invalid = aim;
  invalid.pulseMax[1] = 1400;
  assert(!interpolateGrid(invalid, 400, 650, true, result));
  puts("Camera aim and finite sort map checks: PASS (no physical hardware)");
}
