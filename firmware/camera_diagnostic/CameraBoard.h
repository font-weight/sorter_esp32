#pragma once
#include <Arduino.h>
#include <esp_camera.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <ColorDetector.h>

// AI-Thinker classic ESP32-CAM ONLY. The user has not confirmed this pin map.
// This header is duplicated in camera_node so both sketches remain standalone.
namespace camera_board {

constexpr int FLASH_LED_PIN = 4; // AI-Thinker ESP32-CAM flash LED (active HIGH).

inline camera_config_t configuration(framesize_t size) {
  camera_config_t c = {};
  c.pin_pwdn = 32; c.pin_reset = -1; c.pin_xclk = 0;
  c.pin_sccb_sda = 26; c.pin_sccb_scl = 27;
  c.pin_d0 = 5; c.pin_d1 = 18; c.pin_d2 = 19; c.pin_d3 = 21;
  c.pin_d4 = 36; c.pin_d5 = 39; c.pin_d6 = 34; c.pin_d7 = 35;
  c.pin_vsync = 25; c.pin_href = 23; c.pin_pclk = 22;
  c.xclk_freq_hz = 20000000;
  c.ledc_timer = LEDC_TIMER_0; c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_RGB565; c.frame_size = size;
  c.jpeg_quality = 12; // Unused in RGB565 mode.
  c.fb_count = 1;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  return c;
}

inline bool start(bool profileConfirmed, framesize_t size) {
  if (!profileConfirmed) {
    Serial.println("CAMERA_DISABLED: confirm actual AI-Thinker pin map in CameraConfig.h");
    return false;
  }
#if !CONFIG_IDF_TARGET_ESP32
  Serial.println("CAMERA_DISABLED: this profile is classic ESP32 only");
  return false;
#endif
  if (!psramFound()) {
    Serial.println("CAMERA_DISABLED: PSRAM missing; verify board and PSRAM setting");
    return false;
  }
  camera_config_t c = configuration(size);
  const esp_err_t error = esp_camera_init(&c);
  if (error != ESP_OK) {
    Serial.printf("CAMERA_INIT_FAILED: 0x%X\n", unsigned(error));
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (!s) { esp_camera_deinit(); Serial.println("SENSOR_MISSING"); return false; }
  Serial.printf("SENSOR PID=0x%X VER=0x%X MIDH=0x%X MIDL=0x%X\n",
                s->id.PID, s->id.VER, s->id.MIDH, s->id.MIDL);
  return true;
}

// The first queued frame may predate the request with fb_count=1. Discard it,
// then require the camera driver's boot-relative timestamp to be >= request.
// Each fb_get can wait for the driver's timeout if camera hardware has failed.
inline camera_fb_t* freshFrame(uint16_t width, uint16_t height) {
  const int64_t requested = esp_timer_get_time();
  camera_fb_t* old = esp_camera_fb_get();
  if (!old) return NULL;
  esp_camera_fb_return(old);
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) return NULL;
    const int64_t stamp = int64_t(fb->timestamp.tv_sec)*1000000 + fb->timestamp.tv_usec;
    if (fb->format == PIXFORMAT_RGB565 && fb->width == width && fb->height == height &&
        fb->len == size_t(width)*height*2 && stamp >= requested) return fb;
    esp_camera_fb_return(fb);
  }
  return NULL;
}

inline void printHardware() {
  Serial.printf("CHIP %s rev=%u cores=%u CPU_MHz=%u reset=%u\n", ESP.getChipModel(),
                ESP.getChipRevision(), ESP.getChipCores(), unsigned(ESP.getCpuFreqMHz()),
                unsigned(esp_reset_reason()));
  Serial.printf("FLASH bytes=%u speed=%u HEAP free=%u min=%u PSRAM found=%u total=%u free=%u\n",
                unsigned(ESP.getFlashChipSize()), unsigned(ESP.getFlashChipSpeed()), unsigned(ESP.getFreeHeap()),
                unsigned(ESP.getMinFreeHeap()), unsigned(psramFound()), unsigned(ESP.getPsramSize()), unsigned(ESP.getFreePsram()));
  Serial.printf("SDK %s\n", ESP.getSdkVersion());
}

inline void printStats(const camera_fb_t* fb) {
  uint64_t r = 0, g = 0, b = 0;
  uint32_t dark = 0, bright = 0;
  uint32_t hash = 2166136261u;
  const size_t pixels = fb->width*fb->height;
  for (size_t i = 0; i < pixels; ++i) {
    const sorter::Rgb rgb = sorter::decodeRgb565BE(fb->buf + 2*i);
    r += rgb.r; g += rgb.g; b += rgb.b;
    if (rgb.r < 12 && rgb.g < 12 && rgb.b < 12) ++dark;
    if (rgb.r > 243 && rgb.g > 243 && rgb.b > 243) ++bright;
    hash = (hash ^ fb->buf[2*i])*16777619u;
    hash = (hash ^ fb->buf[2*i+1])*16777619u;
  }
  Serial.printf("FRAME %ux%u bytes=%u meanRGB=%.1f,%.1f,%.1f dark_pct=%.1f bright_pct=%.1f hash=%08X\n",
                unsigned(fb->width), unsigned(fb->height), unsigned(fb->len),
                double(r)/pixels, double(g)/pixels, double(b)/pixels,
                100.0*dark/pixels, 100.0*bright/pixels, unsigned(hash));
}

inline void captureRaw(uint16_t width, uint16_t height) {
  camera_fb_t* fb = freshFrame(width, height);
  if (!fb) { Serial.println("ERROR CAPTURE_FAILED"); return; }
  Serial.printf("RGB565BE %u %u %u\n", width, height, unsigned(fb->len));
  Serial.write(fb->buf, fb->len);
  Serial.print("\nEND\n");
  Serial.flush();
  esp_camera_fb_return(fb);
}

// A bounded USB console. After overflow discard through newline, never execute
// a truncated command (for example a long line starting with CAPTURE).
class Console {
 public:
  Console() : size_(0), overflow_(false) { line_[0] = '\0'; }
  bool poll() {
    while (Serial.available()) {
      const char ch = char(Serial.read());
      if (ch == '\r') continue;
      if (ch == '\n') {
        if (overflow_) { size_ = 0; overflow_ = false; Serial.println("ERROR COMMAND_TOO_LONG"); continue; }
        line_[size_] = '\0'; size_ = 0;
        return line_[0] != '\0';
      }
      if (size_ + 1 >= sizeof(line_)) overflow_ = true;
      else if (!overflow_) line_[size_++] = ch;
    }
    return false;
  }
  const char* line() const { return line_; }
 private:
  char line_[48]; size_t size_; bool overflow_;
};
} // namespace camera_board
