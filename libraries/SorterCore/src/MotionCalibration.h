#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>

namespace sorter {

// PWM calibration, not inverse kinematics. Four channels: base, shoulder,
// linkage/elbow, gripper. Cartesian coordinates use 0.1 mm throughout.
struct JointPose { uint16_t us[4]; };
struct ToolPoint { int32_t x10, y10, z10; };
struct CalibratedPose { JointPose joints; ToolPoint tip; bool measured; };
struct GridCorner { JointPose lowOpen, highOpen; bool measuredLow, measuredHigh; };

struct MotionCalibration {
  uint32_t calibrationId;
  bool geometryConfirmed, routesVerified, parkSupported;
  int32_t pickMinX10, pickMaxX10, pickMinY10, pickMaxY10;
  int32_t lowZ10, highZ10;
  // Grid corner order: (minX,minY), (maxX,minY), (minX,maxY), (maxX,maxY).
  GridCorner grid[4];
  // Bounds cover pick area, transit, bins and park. Z is the lowest finger tip.
  int32_t minX10, maxX10, minY10, maxY10, minZ10, maxZ10;
  uint16_t pulseMin[4], pulseMax[4], openUs, closedUs;
  uint16_t maxRateUsPerSecond[4];
  uint16_t maxAccelerationUsPerSecond2[4];
  CalibratedPose parkOpen, hubOpen, binHighOpen[3], binDropOpen[3];
};

inline bool poseWithinLimits(const JointPose& p, const MotionCalibration& c) {
  for (size_t j = 0; j < 4; ++j)
    if (p.us[j] < c.pulseMin[j] || p.us[j] > c.pulseMax[j]) return false;
  return true;
}
inline bool pointWithinBounds(const ToolPoint& p, const MotionCalibration& c) {
  return p.x10 >= c.minX10 && p.x10 <= c.maxX10 &&
         p.y10 >= c.minY10 && p.y10 <= c.maxY10 &&
         p.z10 >= c.minZ10 && p.z10 <= c.maxZ10;
}
inline bool inPickArea(int32_t x10, int32_t y10, const MotionCalibration& c) {
  return x10 >= c.pickMinX10 && x10 <= c.pickMaxX10 &&
         y10 >= c.pickMinY10 && y10 <= c.pickMaxY10;
}
inline bool knownPoseValid(const CalibratedPose& p, const MotionCalibration& c,
                           bool requireMeasured) {
  return (!requireMeasured || p.measured) && poseWithinLimits(p.joints, c) &&
         pointWithinBounds(p.tip, c) && p.joints.us[3] == c.openUs;
}

inline bool calibrationValid(const MotionCalibration& c, bool requireMeasured) {
  if (!c.calibrationId || c.pickMinX10 >= c.pickMaxX10 ||
      c.pickMinY10 >= c.pickMaxY10 || c.lowZ10 >= c.highZ10 ||
      c.minX10 >= c.maxX10 || c.minY10 >= c.maxY10 ||
      c.minZ10 < 0 || c.minZ10 >= c.maxZ10) return false;
  // Bound numbers to practical tabletop values and keep interpolation arithmetic safe.
  if (c.minX10 < -10000 || c.maxX10 > 10000 || c.minY10 < -10000 ||
      c.maxY10 > 10000 || c.maxZ10 > 10000) return false;
  if (requireMeasured && (!c.geometryConfirmed || !c.routesVerified || !c.parkSupported))
    return false;
  for (size_t j = 0; j < 4; ++j) {
    if (c.pulseMin[j] < 500 || c.pulseMax[j] > 2500 ||
        c.pulseMin[j] >= c.pulseMax[j] || !c.maxRateUsPerSecond[j] ||
        c.maxRateUsPerSecond[j] > 1000 || !c.maxAccelerationUsPerSecond2[j] ||
        c.maxAccelerationUsPerSecond2[j] > 3000) return false;
  }
  if (c.openUs == c.closedUs || c.openUs < c.pulseMin[3] ||
      c.openUs > c.pulseMax[3] || c.closedUs < c.pulseMin[3] ||
      c.closedUs > c.pulseMax[3]) return false;
  for (size_t i = 0; i < 4; ++i) {
    const GridCorner& g = c.grid[i];
    if (requireMeasured && (!g.measuredLow || !g.measuredHigh)) return false;
    if (!poseWithinLimits(g.lowOpen, c) || !poseWithinLimits(g.highOpen, c) ||
        g.lowOpen.us[3] != c.openUs || g.highOpen.us[3] != c.openUs) return false;
    ToolPoint p = {(i & 1) ? c.pickMaxX10 : c.pickMinX10,
                  (i & 2) ? c.pickMaxY10 : c.pickMinY10, c.lowZ10};
    if (!pointWithinBounds(p, c)) return false;
    p.z10 = c.highZ10;
    if (!pointWithinBounds(p, c)) return false;
  }
  if (!knownPoseValid(c.parkOpen, c, requireMeasured) ||
      !knownPoseValid(c.hubOpen, c, requireMeasured)) return false;
  for (size_t i = 0; i < 3; ++i)
    if (!knownPoseValid(c.binHighOpen[i], c, requireMeasured) ||
        !knownPoseValid(c.binDropOpen[i], c, requireMeasured) ||
        c.binHighOpen[i].tip.z10 <= c.binDropOpen[i].tip.z10) return false;
  return true;
}

inline bool interpolateGrid(const MotionCalibration& c, int32_t x10, int32_t y10,
                            bool high, CalibratedPose& result) {
  if (!inPickArea(x10, y10, c) || c.pickMaxX10 <= c.pickMinX10 ||
      c.pickMaxY10 <= c.pickMinY10) return false;
  const double u = (double(x10) - c.pickMinX10) / (double(c.pickMaxX10) - c.pickMinX10);
  const double v = (double(y10) - c.pickMinY10) / (double(c.pickMaxY10) - c.pickMinY10);
  const double w[4] = {(1-u)*(1-v), u*(1-v), (1-u)*v, u*v};
  result.measured = false; // Interior values are interpolated, never called measurements.
  result.tip = {x10, y10, high ? c.highZ10 : c.lowZ10};
  for (size_t j = 0; j < 4; ++j) {
    double value = 0;
    for (size_t k = 0; k < 4; ++k)
      value += w[k] * (high ? c.grid[k].highOpen.us[j] : c.grid[k].lowOpen.us[j]);
    result.joints.us[j] = uint16_t(value + 0.5);
  }
  return poseWithinLimits(result.joints, c) && pointWithinBounds(result.tip, c);
}

// Deliberately invented demonstration numbers. Never use these as physical calibration.
inline MotionCalibration exampleVirtualCalibration() {
  MotionCalibration c = {};
  c.calibrationId = 1;
  c.pickMinX10 = -300; c.pickMaxX10 = 300;
  c.pickMinY10 = 800; c.pickMaxY10 = 1200;
  c.lowZ10 = 30; c.highZ10 = 500;
  c.minX10 = -1000; c.maxX10 = 1000;
  c.minY10 = -200; c.maxY10 = 1600; c.minZ10 = 20; c.maxZ10 = 1200;
  c.openUs = 1350; c.closedUs = 1550;
  for (size_t j = 0; j < 4; ++j) {
    c.pulseMin[j] = 1000; c.pulseMax[j] = 2000;
    c.maxRateUsPerSecond[j] = 250; c.maxAccelerationUsPerSecond2[j] = 500;
  }
  const JointPose low[4] = {{{1400,1600,1500,1350}}, {{1600,1600,1500,1350}},
                           {{1400,1700,1450,1350}}, {{1600,1700,1450,1350}}};
  const JointPose high[4] = {{{1400,1400,1500,1350}}, {{1600,1400,1500,1350}},
                            {{1400,1500,1450,1350}}, {{1600,1500,1450,1350}}};
  for (size_t i = 0; i < 4; ++i) { c.grid[i].lowOpen = low[i]; c.grid[i].highOpen = high[i]; }
  c.parkOpen = {{{1500,1300,1500,1350}}, {0,400,600}, false};
  c.hubOpen = {{{1500,1400,1500,1350}}, {0,700,500}, false};
  for (size_t i = 0; i < 3; ++i) {
    c.binHighOpen[i] = {{{uint16_t(1100 + 400*i),1400,1550,1350}},
                        {int32_t(-800 + int32_t(i)*800),500,500}, false};
    c.binDropOpen[i] = c.binHighOpen[i];
    c.binDropOpen[i].joints.us[1] = 1500; c.binDropOpen[i].tip.z10 = 300;
  }
  return c;
}

} // namespace sorter
