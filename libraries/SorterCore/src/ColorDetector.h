#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "PartClasses.h"

namespace sorter {

// The esp32-camera RGB565 buffer stores the high byte first. Never cast its
// bytes to uint16_t*: ESP32 and most development PCs are little-endian.
struct Rgb { uint8_t r, g, b; };
inline Rgb decodeRgb565BE(const uint8_t* p) {
  const uint16_t word = (uint16_t(p[0]) << 8) | p[1];
  const uint8_t r = uint8_t((word >> 11) & 31);
  const uint8_t g = uint8_t((word >> 5) & 63);
  const uint8_t b = uint8_t(word & 31);
  return Rgb{uint8_t((r << 3) | (r >> 2)),
             uint8_t((g << 2) | (g >> 4)),
             uint8_t((b << 3) | (b >> 2))};
}

// Inclusive RGB888 thresholds after bit-replication expansion from RGB565.
// classId=1 (red) or 3 (blue); zero is background, ambiguity or unknown color.
struct ColorRange {
  uint8_t classId, rMin, rMax, gMin, gMax, bMin, bMax, minChroma;
};
inline bool validColorRanges(const ColorRange* ranges, size_t count) {
  if (!ranges || count == 0 || count > kColorCount) return false;
  uint8_t used = 0;
  for (size_t i = 0; i < count; ++i) {
    const ColorRange& t = ranges[i];
    if (!isSupportedClass(t.classId) || (used & (1 << t.classId)) ||
        t.rMin > t.rMax || t.gMin > t.gMax || t.bMin > t.bMax) return false;
    used |= uint8_t(1 << t.classId);
  }
  return true;
}

constexpr int COLOR_DOMINANCE_DELTA = 15;

inline uint8_t classifyRgb(Rgb rgb, const ColorRange* ranges, size_t count,
                           bool* ambiguous = NULL) {
  if (ambiguous) *ambiguous = false;
  uint8_t low = rgb.r, high = rgb.r;
  if (rgb.g < low) low = rgb.g;
  if (rgb.b < low) low = rgb.b;
  if (rgb.g > high) high = rgb.g;
  if (rgb.b > high) high = rgb.b;
  uint8_t result = 0;
  for (size_t i = 0; i < count; ++i) {
    const ColorRange& t = ranges[i];
    if (!isSupportedClass(t.classId)) continue;
    if (rgb.r >= t.rMin && rgb.r <= t.rMax &&
        rgb.g >= t.gMin && rgb.g <= t.gMax &&
        rgb.b >= t.bMin && rgb.b <= t.bMax && high - low >= t.minChroma) {
      if (result) {
        if (ambiguous) *ambiguous = true;
        return 0;
      }
      result = t.classId;
    }
  }
  // Reject overlapping RGB ranges before checking blue dominance.
  // Red keeps its existing RGB/chroma conditions; green is never classified.
  if (result == kBlueClassId &&
      !(rgb.b > int(rgb.g) + COLOR_DOMINANCE_DELTA &&
        rgb.b > int(rgb.r) + COLOR_DOMINANCE_DELTA)) return 0;
  return result;
}

// Half-open ROI: [left,right) x [top,bottom). Coordinates are full-frame pixels.
struct PixelRoi { uint16_t left, top, right, bottom; };
struct DetectorConfig {
  const ColorRange* ranges;
  size_t rangeCount;
  PixelRoi roi;
  uint32_t minPixels, maxPixels;
  bool rejectRoiEdge;
};
struct PixelBlob {
  uint8_t classId;
  uint32_t pixels;
  double centerX, centerY;
  uint16_t left, top, right, bottom; // Inclusive bounding box.
};
struct DetectorStats {
  uint32_t matchedPixels, ambiguousPixels, components;
  uint32_t rejectedSmall, rejectedLarge, rejectedEdge;
};
enum class DetectionStatus { Ok, BadImage, BadConfig, WorkspaceTooSmall, TooManyObjects };
struct DetectionResult {
  DetectionStatus status;
  size_t count;
  DetectorStats stats;
};

// Caller allocates once (PSRAM on ESP32): 1 + 4 bytes per pixel. Neither this
// routine nor its flood fill allocates memory or uses a large stack array.
struct DetectorWorkspace {
  uint8_t* labels;
  size_t labelCapacity;
  uint32_t* queue;
  size_t queueCapacity;
};

inline DetectionResult detectRgb565BE(const uint8_t* image, size_t imageBytes,
                                      uint16_t width, uint16_t height,
                                      const DetectorConfig& cfg,
                                      DetectorWorkspace work,
                                      PixelBlob* output, size_t outputCapacity) {
  DetectionResult result = {DetectionStatus::Ok, 0, {0, 0, 0, 0, 0, 0}};
  if (!image || !width || !height || width > 320 || height > 240 ||
      imageBytes != size_t(width) * height * 2) {
    result.status = DetectionStatus::BadImage;
    return result;
  }
  const size_t pixels = size_t(width) * height;
  if (!validColorRanges(cfg.ranges, cfg.rangeCount) || !output || !outputCapacity ||
      cfg.minPixels == 0 || cfg.maxPixels < cfg.minPixels || cfg.maxPixels > pixels ||
      cfg.roi.left >= cfg.roi.right || cfg.roi.top >= cfg.roi.bottom ||
      cfg.roi.right > width || cfg.roi.bottom > height) {
    result.status = DetectionStatus::BadConfig;
    return result;
  }
  if (!work.labels || !work.queue || work.labelCapacity < pixels || work.queueCapacity < pixels) {
    result.status = DetectionStatus::WorkspaceTooSmall;
    return result;
  }
  memset(work.labels, 0, pixels);
  for (uint16_t y = cfg.roi.top; y < cfg.roi.bottom; ++y) {
    for (uint16_t x = cfg.roi.left; x < cfg.roi.right; ++x) {
      const size_t i = size_t(y) * width + x;
      bool ambiguous = false;
      const uint8_t label = classifyRgb(decodeRgb565BE(image + 2*i), cfg.ranges,
                                        cfg.rangeCount, &ambiguous);
      work.labels[i] = label;
      if (label) ++result.stats.matchedPixels;
      if (ambiguous) ++result.stats.ambiguousPixels;
    }
  }
  for (uint16_t y = cfg.roi.top; y < cfg.roi.bottom; ++y) {
    for (uint16_t x = cfg.roi.left; x < cfg.roi.right; ++x) {
      const uint32_t first = uint32_t(y) * width + x;
      const uint8_t color = work.labels[first];
      if (!color) continue;
      ++result.stats.components;
      size_t head = 0, tail = 0;
      work.queue[tail++] = first;
      work.labels[first] = 0; // Mark on enqueue; each pixel enters once.
      uint64_t sumX = 0, sumY = 0;
      PixelBlob blob = {color, 0, 0, 0, x, y, x, y};
      while (head < tail) {
        const uint32_t index = work.queue[head++];
        const uint16_t px = uint16_t(index % width), py = uint16_t(index / width);
        ++blob.pixels;
        sumX += px; sumY += py;
        if (px < blob.left) blob.left = px;
        if (px > blob.right) blob.right = px;
        if (py < blob.top) blob.top = py;
        if (py > blob.bottom) blob.bottom = py;
        // Eight-connected components prevent a diagonal edge from fragmenting.
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (!dx && !dy) continue;
            const int nx = int(px) + dx, ny = int(py) + dy;
            if (nx < cfg.roi.left || nx >= cfg.roi.right ||
                ny < cfg.roi.top || ny >= cfg.roi.bottom) continue;
            const uint32_t next = uint32_t(ny) * width + uint32_t(nx);
            if (work.labels[next] != color) continue;
            // Since each pixel is enqueued once, tail <= width*height.
            work.labels[next] = 0;
            work.queue[tail++] = next;
          }
        }
      }
      if (blob.pixels < cfg.minPixels) { ++result.stats.rejectedSmall; continue; }
      if (blob.pixels > cfg.maxPixels) { ++result.stats.rejectedLarge; continue; }
      if (cfg.rejectRoiEdge &&
          (blob.left == cfg.roi.left || blob.top == cfg.roi.top ||
           blob.right + 1 == cfg.roi.right || blob.bottom + 1 == cfg.roi.bottom)) {
        ++result.stats.rejectedEdge; continue;
      }
      if (result.count == outputCapacity) {
        // A truncated scene could hide an obstruction: do not publish a subset.
        result.status = DetectionStatus::TooManyObjects;
        result.count = 0;
        return result;
      }
      blob.centerX = double(sumX) / blob.pixels;
      blob.centerY = double(sumY) / blob.pixels;
      output[result.count++] = blob;
    }
  }
  return result;
}

} // namespace sorter
