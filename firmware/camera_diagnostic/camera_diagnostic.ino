#include <Arduino.h>
#include "CameraConfig.h"
#include "CameraBoard.h"

bool cameraReady = false;
camera_board::Console console;

void setup() {
  if (CAMERA_PROFILE_CONFIRMED) {
    pinMode(camera_board::FLASH_LED_PIN, OUTPUT);
    digitalWrite(camera_board::FLASH_LED_PIN, LOW);
  }
  Serial.begin(USB_BAUD);
  Serial.setDebugOutput(false);
  delay(400);
  Serial.println("SORTER CAMERA DIAGNOSTIC; no motor control; RGB565BE 160x120");
  camera_board::printHardware();
  cameraReady = camera_board::start(CAMERA_PROFILE_CONFIRMED, FRAMESIZE_QQVGA);
  Serial.println("Commands: STATUS, STATS, TEST, CAPTURE, LED ON, LED OFF, HELP (newline; 460800 baud)");
}

void loop() {
  if (!console.poll()) { delay(2); return; }
  const char* cmd = console.line();
  if (!strcmp(cmd, "HELP")) {
    Serial.println("STATUS=hardware; STATS=one fresh frame; TEST=20 frames; CAPTURE=raw binary; LED ON/OFF=flash LED");
    return;
  }
  if (!strcmp(cmd, "STATUS")) {
    camera_board::printHardware();
    Serial.printf("CAMERA_READY=%u PROFILE_CONFIRMED=%u\n", cameraReady, CAMERA_PROFILE_CONFIRMED);
    return;
  }
  if (!strcmp(cmd, "LED ON") || !strcmp(cmd, "LED OFF")) {
    if (!CAMERA_PROFILE_CONFIRMED) { Serial.println("ERROR LED_DISABLED"); return; }
    digitalWrite(camera_board::FLASH_LED_PIN, !strcmp(cmd, "LED ON") ? HIGH : LOW);
    Serial.print("OK "); Serial.println(cmd);
    return;
  }
  if (!cameraReady) { Serial.println("ERROR CAMERA_DISABLED"); return; }
  if (!strcmp(cmd, "CAPTURE")) { camera_board::captureRaw(160, 120); return; }
  if (!strcmp(cmd, "STATS") || !strcmp(cmd, "TEST")) {
    const unsigned requested = !strcmp(cmd, "TEST") ? 20 : 1;
    unsigned ok = 0;
    const uint32_t started = millis(), heapBefore = ESP.getFreeHeap();
    for (unsigned i = 0; i < requested; ++i) {
      camera_fb_t* fb = camera_board::freshFrame(160, 120);
      if (!fb) { Serial.printf("FRAME_FAILED index=%u\n", i); break; }
      camera_board::printStats(fb);
      esp_camera_fb_return(fb);
      ++ok;
      delay(2);
    }
    Serial.printf("TEST_RESULT frames=%u/%u elapsed_ms=%u heap_before=%u heap_after=%u\n",
                  ok, requested, unsigned(millis()-started), unsigned(heapBefore), unsigned(ESP.getFreeHeap()));
    Serial.println("Frame receipt does not validate image quality, colors, coordinates or power stability.");
    return;
  }
  Serial.println("ERROR UNKNOWN_COMMAND");
}
