#pragma once
#include <stdint.h>
namespace fakehw { inline uint32_t randomValue=101,randomCalls=0; }
inline uint32_t esp_random(){++fakehw::randomCalls;return fakehw::randomValue;}
