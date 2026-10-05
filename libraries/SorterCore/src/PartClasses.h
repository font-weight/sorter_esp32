#pragma once
#include <stddef.h>
#include <stdint.h>

namespace sorter {

// Keep the existing wire IDs: red=1, blue=3. The former green ID=2 is invalid.
constexpr uint8_t kRedClassId = 1;
constexpr uint8_t kBlueClassId = 3;
constexpr size_t kColorCount = 2;
constexpr uint8_t kColorClassIds[kColorCount] = {kRedClassId, kBlueClassId};

inline bool isSupportedClass(uint32_t classId) {
  return classId == kRedClassId || classId == kBlueClassId;
}

// Array indices: first bin is red, second bin is blue; 255 means invalid.
inline uint8_t classBinIndex(uint8_t classId) {
  return classId == kRedClassId ? 0 : classId == kBlueClassId ? 1 : 255;
}

} // namespace sorter
