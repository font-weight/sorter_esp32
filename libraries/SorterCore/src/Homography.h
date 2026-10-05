#pragma once

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>

namespace sorter {

struct Point2 { double x, y; };

// Convex calibration support (clockwise or counterclockwise), at most 16
// vertices. The matrix transforms camera pixels to table millimetres.
class Homography {
 public:
  Homography() : count_(0), valid_(false) {}
  bool configure(const double matrix[9], const Point2* support, size_t count) {
    valid_ = false;
    count_ = 0;
    if (!matrix || !support || count < 3 || count > 16) return false;
    double scale = 0;
    for (size_t i = 0; i < 9; ++i) {
      if (!isfinite(matrix[i])) return false;
      if (fabs(matrix[i]) > scale) scale = fabs(matrix[i]);
    }
    if (scale == 0) return false;
    for (size_t i = 0; i < 9; ++i) m_[i] = matrix[i] / scale;
    const double det = m_[0]*(m_[4]*m_[8] - m_[5]*m_[7])
                     -m_[1]*(m_[3]*m_[8] - m_[5]*m_[6])
                     +m_[2]*(m_[3]*m_[7] - m_[4]*m_[6]);
    if (!isfinite(det) || fabs(det) < 1e-14) return false;
    double winding = 0;
    double denominatorSign = 0;
    for (size_t i = 0; i < count; ++i) {
      if (!isfinite(support[i].x) || !isfinite(support[i].y)) return false;
    }
    for (size_t i = 0; i < count; ++i) {
      const Point2& a = support[i];
      const Point2& b = support[(i + 1) % count];
      const Point2& c = support[(i + 2) % count];
      const double cross = (b.x-a.x)*(c.y-b.y) - (b.y-a.y)*(c.x-b.x);
      if (!isfinite(cross) || fabs(cross) < 1e-8) return false;
      if (!winding) winding = cross;
      else if (cross*winding <= 0) return false;
      const double denominator = m_[6]*a.x + m_[7]*a.y + m_[8];
      if (!isfinite(denominator) || fabs(denominator) < 1e-10) return false;
      if (!denominatorSign) denominatorSign = denominator;
      else if (denominator * denominatorSign <= 0) return false;
      vertices_[i] = a;
    }
    // Reject self-intersecting star polygons too: every vertex must lie in
    // every inward half-plane, not just have the same local turn direction.
    for (size_t i = 0; i < count; ++i) {
      const Point2& a = support[i];
      const Point2& b = support[(i + 1) % count];
      for (size_t j = 0; j < count; ++j) {
        const Point2& p = support[j];
        const double cross = (b.x-a.x)*(p.y-a.y) - (b.y-a.y)*(p.x-a.x);
        if (cross * winding < -1e-8) return false;
      }
    }
    count_ = count;
    valid_ = true;
    return true;
  }
  bool valid() const { return valid_; }
  bool contains(double x, double y) const {
    if (!valid_ || !isfinite(x) || !isfinite(y)) return false;
    double sign = 0;
    for (size_t i = 0; i < count_; ++i) {
      const Point2& a = vertices_[i];
      const Point2& b = vertices_[(i + 1) % count_];
      const double cross = (b.x-a.x)*(y-a.y) - (b.y-a.y)*(x-a.x);
      if (!isfinite(cross)) return false;
      if (fabs(cross) <= 1e-8) continue;
      if (!sign) sign = cross;
      else if (cross*sign < 0) return false;
    }
    return true;
  }
  bool map(double x, double y, double& mmX, double& mmY) const {
    if (!contains(x, y)) return false;
    const double denominator = m_[6]*x + m_[7]*y + m_[8];
    if (!isfinite(denominator) || fabs(denominator) < 1e-10) return false;
    const double outX = (m_[0]*x + m_[1]*y + m_[2])/denominator;
    const double outY = (m_[3]*x + m_[4]*y + m_[5])/denominator;
    if (!isfinite(outX) || !isfinite(outY)) return false;
    mmX = outX; mmY = outY;
    return true;
  }
  bool mapTenths(double x, double y, int32_t& x10, int32_t& y10) const {
    double mmX, mmY;
    if (!map(x, y, mmX, mmY)) return false;
    const double sx = round(mmX*10), sy = round(mmY*10);
    if (!isfinite(sx) || !isfinite(sy) || sx < INT32_MIN || sx > INT32_MAX ||
        sy < INT32_MIN || sy > INT32_MAX) return false;
    x10 = int32_t(sx); y10 = int32_t(sy);
    return true;
  }
 private:
  double m_[9];
  Point2 vertices_[16];
  size_t count_;
  bool valid_;
};

} // namespace sorter
