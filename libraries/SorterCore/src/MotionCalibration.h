#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "PartClasses.h"

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
  // Optional measured interior nodes. Disabled levels retain the original
  // bilinear map; enabled levels use four triangles with this node as their hub.
  bool centerLowEnabled, centerHighEnabled;
  CalibratedPose centerLowOpen, centerHighOpen;
  // Bounds cover pick area, transit, bins and park. Z is the lowest finger tip.
  int32_t minX10, maxX10, minY10, maxY10, minZ10, maxZ10;
  uint16_t pulseMin[4], pulseMax[4], openUs, closedUs;
  uint16_t maxRateUsPerSecond[4];
  uint16_t maxAccelerationUsPerSecond2[4];
  // Bin 0: red (class 1); bin 1: blue (class 3).
  CalibratedPose parkOpen, hubOpen, binHighOpen[kColorCount], binDropOpen[kColorCount];
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

inline bool centerLevelValid(const MotionCalibration& c, bool high) {
  if (!(high ? c.centerHighEnabled : c.centerLowEnabled)) return true;
  const CalibratedPose& center = high ? c.centerHighOpen : c.centerLowOpen;
  if (!knownPoseValid(center, c, true) ||
      center.tip.x10 <= c.pickMinX10 || center.tip.x10 >= c.pickMaxX10 ||
      center.tip.y10 <= c.pickMinY10 || center.tip.y10 >= c.pickMaxY10 ||
      (high ? center.tip.z10 <= c.lowZ10 : center.tip.z10 >= c.highZ10)) return false;
  for (size_t i = 0; i < 4; ++i) {
    const GridCorner& g = c.grid[i];
    const JointPose& pose = high ? g.highOpen : g.lowOpen;
    const ToolPoint point = {(i & 1) ? c.pickMaxX10 : c.pickMinX10,
                            (i & 2) ? c.pickMaxY10 : c.pickMinY10,
                            high ? c.highZ10 : c.lowZ10};
    if (!(high ? g.measuredHigh : g.measuredLow) || !poseWithinLimits(pose, c) ||
        pose.us[3] != c.openUs || !pointWithinBounds(point, c)) return false;
  }
  return true;
}

inline bool centersValid(const MotionCalibration& c, bool requireMeasured) {
  // A partly refined map is useful for virtual commissioning, but cannot
  // authorize physical operation with an unrefined lower or upper level.
  if (requireMeasured && c.centerLowEnabled != c.centerHighEnabled) return false;
  if (!centerLevelValid(c, false) || !centerLevelValid(c, true)) return false;
  return !(c.centerLowEnabled && c.centerHighEnabled) ||
    (c.centerLowOpen.tip.x10 == c.centerHighOpen.tip.x10 &&
     c.centerLowOpen.tip.y10 == c.centerHighOpen.tip.y10 &&
     c.centerLowOpen.tip.z10 < c.centerHighOpen.tip.z10);
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
  if (!centersValid(c, requireMeasured)) return false;
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
  for (size_t i = 0; i < kColorCount; ++i)
    if (!knownPoseValid(c.binHighOpen[i], c, requireMeasured) ||
        !knownPoseValid(c.binDropOpen[i], c, requireMeasured) ||
        c.binHighOpen[i].tip.z10 <= c.binDropOpen[i].tip.z10) return false;
  return true;
}

inline bool interpolateGrid(const MotionCalibration& c, int32_t x10, int32_t y10,
                            bool high, CalibratedPose& result) {
  if (!inPickArea(x10, y10, c) || c.pickMaxX10 <= c.pickMinX10 ||
      c.pickMaxY10 <= c.pickMinY10) return false;
  // Validate enabled vertices even when called without calibrationValid().
  if (!centersValid(c, false)) return false;
  const double u = (double(x10) - c.pickMinX10) / (double(c.pickMaxX10) - c.pickMinX10);
  const double v = (double(y10) - c.pickMinY10) / (double(c.pickMaxY10) - c.pickMinY10);
  // Keep the exact legacy calculation on rectangle edges, including its
  // integer rounding. Their center weight is mathematically zero.
  if ((high ? c.centerHighEnabled : c.centerLowEnabled) &&
      x10 != c.pickMinX10 && x10 != c.pickMaxX10 &&
      y10 != c.pickMinY10 && y10 != c.pickMaxY10) {
    const CalibratedPose& center = high ? c.centerHighOpen : c.centerLowOpen;
    const double cu = (double(center.tip.x10) - c.pickMinX10) /
                      (double(c.pickMaxX10) - c.pickMinX10);
    const double cv = (double(center.tip.y10) - c.pickMinY10) /
                      (double(c.pickMaxY10) - c.pickMinY10);
    const double cornerU[4] = {0,1,0,1}, cornerV[4] = {0,0,1,1};
    const uint8_t pairs[4][2] = {{0,1},{1,3},{3,2},{2,0}};
    for (size_t i = 0; i < 4; ++i) {
      const uint8_t a = pairs[i][0], b = pairs[i][1];
      const double denominator = (cornerV[b]-cv)*(cornerU[a]-cu) +
                                 (cu-cornerU[b])*(cornerV[a]-cv);
      if (denominator == 0) return false;
      double weights[3] = {
        ((cornerV[b]-cv)*(u-cu) + (cu-cornerU[b])*(v-cv)) / denominator,
        ((cv-cornerV[a])*(u-cu) + (cornerU[a]-cu)*(v-cv)) / denominator, 0};
      weights[2] = 1-weights[0]-weights[1];
      if (weights[0] < -1e-9 || weights[1] < -1e-9 || weights[2] < -1e-9) continue;
      // Clamp only floating-point seam noise, then normalize. The resulting
      // convex weights keep PWM and Z inside their measured vertex ranges.
      double sum = 0;
      for (size_t k = 0; k < 3; ++k) {
        if (weights[k] < 0) weights[k] = 0;
        sum += weights[k];
      }
      for (size_t k = 0; k < 3; ++k) weights[k] /= sum;
      const JointPose& poseA = high ? c.grid[a].highOpen : c.grid[a].lowOpen;
      const JointPose& poseB = high ? c.grid[b].highOpen : c.grid[b].lowOpen;
      result.measured = false;
      const double z = (weights[0]+weights[1])*(high ? c.highZ10 : c.lowZ10) +
                       weights[2]*center.tip.z10;
      result.tip = {x10, y10, int32_t(z + 0.5)};
      for (size_t j = 0; j < 4; ++j)
        result.joints.us[j] = uint16_t(weights[0]*poseA.us[j] +
          weights[1]*poseB.us[j] + weights[2]*center.joints.us[j] + 0.5);
      return poseWithinLimits(result.joints, c) && pointWithinBounds(result.tip, c);
    }
    return false;
  }
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

} // namespace sorter
