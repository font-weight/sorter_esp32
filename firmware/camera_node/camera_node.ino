#include <Arduino.h>
#include <esp_heap_caps.h>
#include <Protocol.h>
#include <ColorDetector.h>
#include <Homography.h>
#include "CameraConfig.h"
#include "GeneratedCalibration.h"
#include "CameraBoard.h"
#include "CameraSession.h"

namespace {
constexpr uint16_t WIDTH = 320, HEIGHT = 240;
constexpr size_t PIXELS = size_t(WIDTH)*HEIGHT;
HardwareSerial cameraUart(1);
camera_board::Console console;
sorter::LineParser wireParser;
sorter::CameraSession requests;
sorter::Homography mapping;
sorter::DetectorWorkspace workspace = {NULL, 0, NULL, 0};
sorter::PixelBlob blobs[sorter::kMaxObjects];
sorter::Packet incoming = {}, outgoing = {};
char encoded[sorter::kMaxFrame];
bool cameraReady = false, uartReady = false, calibrationReady = false;
uint32_t cameraBoot = 0;
}

bool configureSensor() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s || s->id.PID != OV2640_PID) {
    Serial.println("ERROR SENSOR_NOT_OV2640: this node profile requires OV2640"); return false;
  }
  if (SENSOR_EXPOSURE < 0 || SENSOR_EXPOSURE > 1200 || SENSOR_GAIN < 0 || SENSOR_GAIN > 30 ||
      SENSOR_WB_MODE < 1 || SENSOR_WB_MODE > 4) return false;
  int error = 0;
  error |= s->set_hmirror(s, SENSOR_HMIRROR);
  error |= s->set_vflip(s, SENSOR_VFLIP);
  error |= s->set_gain_ctrl(s, 0);
  error |= s->set_agc_gain(s, SENSOR_GAIN);
  error |= s->set_exposure_ctrl(s, 0);
  error |= s->set_aec2(s, 0);
  error |= s->set_aec_value(s, SENSOR_EXPOSURE);
  error |= s->set_whitebal(s, 0);
  error |= s->set_awb_gain(s, 1);
  error |= s->set_wb_mode(s, SENSOR_WB_MODE);
  error |= s->set_brightness(s, 0);
  error |= s->set_contrast(s, 0);
  error |= s->set_saturation(s, 0);
  return error == 0;
}

void printStatus() {
  camera_board::printHardware();
  Serial.printf("CAMERA=%u UART=%u VISION_CONFIRMED=%u CALIBRATION=%u ID=%u BOOT=%u PARSE_ERRORS=%u\n",
                cameraReady, uartReady, VISION_CONFIG_CONFIRMED, calibrationReady,
                unsigned(sorter_calibration::CALIBRATION_ID), unsigned(cameraBoot),
                unsigned(wireParser.errors()));
  Serial.printf("SENSOR exposure=%d gain=%d wb=%d hmirror=%u vflip=%u\n",
                SENSOR_EXPOSURE, SENSOR_GAIN, SENSOR_WB_MODE, SENSOR_HMIRROR, SENSOR_VFLIP);
  Serial.printf("WORKSPACE bytes=%u ready=%u RGB565BE %ux%u\n", unsigned(PIXELS*5),
                workspace.labels != NULL && workspace.queue != NULL, WIDTH, HEIGHT);
}

void emitReply(bool cache) {
  if (!sorter::encodePacket(outgoing, encoded, sizeof(encoded))) {
    outgoing.type = 'E'; strcpy(outgoing.payload, "ENCODE_FAILED");
    if (!sorter::encodePacket(outgoing, encoded, sizeof(encoded))) return;
  }
  if (cache) requests.remember(encoded);
  cameraUart.print(encoded);
}

void emitError(const sorter::Packet& request, const char* reason, bool cache) {
  outgoing.type = 'E'; outgoing.session = request.session; outgoing.seq = request.seq;
  snprintf(outgoing.payload, sizeof(outgoing.payload), "%s", reason);
  emitReply(cache);
}

sorter::DetectionResult scanFrame() {
  sorter::DetectionResult result = {sorter::DetectionStatus::BadImage, 0, {0,0,0,0,0,0}};
  if (!cameraReady) return result;
  camera_fb_t* fb = camera_board::freshFrame(WIDTH, HEIGHT);
  if (!fb) return result;
  result = sorter::detectRgb565BE(fb->buf, fb->len, WIDTH, HEIGHT, DETECTOR_CONFIG,
                                 workspace, blobs, sorter::kMaxObjects);
  esp_camera_fb_return(fb);
  return result;
}

void handleRequest(const sorter::Packet& request) {
  if (request.type != 'Q') { emitError(request, "BAD_TYPE", false); return; }
  uint32_t wantedCalibration = 0;
  if (!sorter::parseU32(request.payload, wantedCalibration) || !wantedCalibration) {
    emitError(request, "BAD_CALIBRATION_ID", false); return;
  }
  const sorter::RequestDecision decision = requests.accept(request.session, request.seq, wantedCalibration);
  if (decision == sorter::RequestDecision::Duplicate) {
    if (requests.response()[0]) cameraUart.print(requests.response());
    else emitError(request, "CACHE_EMPTY", false);
    return;
  }
  if (decision != sorter::RequestDecision::New) {
    emitError(request, decision == sorter::RequestDecision::Conflict ? "CONFLICT" :
                       decision == sorter::RequestDecision::Stale ? "STALE" : "BAD_REQUEST", false);
    return;
  }
  if (!cameraReady) { emitError(request, "CAMERA_NOT_READY", true); return; }
  if (!VISION_CONFIG_CONFIRMED) { emitError(request, "VISION_NOT_CALIBRATED", true); return; }
  if (!calibrationReady) { emitError(request, "CALIBRATION_INVALID", true); return; }
  if (wantedCalibration != sorter_calibration::CALIBRATION_ID) {
    emitError(request, "CALIBRATION_MISMATCH", true); return;
  }
  const uint32_t started = millis();
  const sorter::DetectionResult found = scanFrame();
  if (found.status != sorter::DetectionStatus::Ok) {
    const char* error = found.status == sorter::DetectionStatus::TooManyObjects ? "TOO_MANY_OBJECTS" :
                        found.status == sorter::DetectionStatus::WorkspaceTooSmall ? "NO_WORKSPACE" :
                        found.status == sorter::DetectionStatus::BadConfig ? "BAD_VISION_CONFIG" : "CAPTURE_FAILED";
    emitError(request, error, true); return;
  }
  sorter::Scene scene = {};
  scene.cameraBoot = cameraBoot; scene.calibrationId = wantedCalibration;
  for (size_t i = 0; i < found.count; ++i) {
    const sorter::PixelBlob& blob = blobs[i];
    if (blob.classId < 1 || blob.classId > 3) continue;
    // Reject even a partly unsupported bounding box. Mapping its centre alone
    // would allow a part extending beyond the measured calibration region.
    if (!mapping.contains(blob.left, blob.top) || !mapping.contains(blob.right, blob.top) ||
        !mapping.contains(blob.right, blob.bottom) || !mapping.contains(blob.left, blob.bottom)) continue;
    sorter::Detection detection = {};
    detection.classId = blob.classId; detection.pixels = blob.pixels;
    if (!mapping.mapTenths(blob.centerX, blob.centerY, detection.x10, detection.y10)) continue;
    scene.objects[scene.count++] = detection;
  }
  outgoing.type = 'D'; outgoing.session = request.session; outgoing.seq = request.seq;
  if (!sorter::encodeScene(scene, outgoing.payload, sizeof(outgoing.payload))) {
    emitError(request, "SCENE_ENCODE_FAILED", true); return;
  }
  emitReply(true);
  Serial.printf("SCENE seq=%u blobs=%u mapped=%u ms=%u rejects_small=%u large=%u edge=%u ambiguous_pixels=%u\n",
                unsigned(request.seq), unsigned(found.count), scene.count, unsigned(millis()-started),
                unsigned(found.stats.rejectedSmall), unsigned(found.stats.rejectedLarge),
                unsigned(found.stats.rejectedEdge), unsigned(found.stats.ambiguousPixels));
}

void handleConsole() {
  if (!console.poll()) return;
  const char* cmd = console.line();
  if (!strcmp(cmd, "STATUS")) { printStatus(); return; }
  if (!strcmp(cmd, "HELP")) {
    Serial.println("STATUS, STATS, SCAN (pixel blobs), CAPTURE (binary), LED ON, LED OFF, HELP; CAPTURE/SCAN blocked if UART enabled");
    return;
  }
  if (!strcmp(cmd, "LED ON") || !strcmp(cmd, "LED OFF")) {
    if (!CAMERA_PROFILE_CONFIRMED) { Serial.println("ERROR LED_DISABLED"); return; }
    digitalWrite(camera_board::FLASH_LED_PIN, !strcmp(cmd, "LED ON") ? HIGH : LOW);
    Serial.print("OK "); Serial.println(cmd);
    return;
  }
  if (!cameraReady) { Serial.println("ERROR CAMERA_DISABLED"); return; }
  // Raw capture takes several seconds over USB-UART. It must not delay a live
  // controller transaction. Use a build with UART disabled for calibration.
  if (uartReady) { Serial.println("ERROR USB_CAMERA_COMMAND_REQUIRES_UART_DISABLED"); return; }
  if (!strcmp(cmd, "CAPTURE")) { camera_board::captureRaw(WIDTH, HEIGHT); return; }
  if (!strcmp(cmd, "STATS")) {
    camera_fb_t* fb = camera_board::freshFrame(WIDTH, HEIGHT);
    if (!fb) { Serial.println("ERROR CAPTURE_FAILED"); return; }
    camera_board::printStats(fb); esp_camera_fb_return(fb); return;
  }
  if (!strcmp(cmd, "SCAN")) {
    const sorter::DetectionResult found = scanFrame();
    Serial.printf("SCAN status=%u count=%u components=%u small=%u large=%u edge=%u ambiguous=%u\n",
                  unsigned(found.status), unsigned(found.count), unsigned(found.stats.components),
                  unsigned(found.stats.rejectedSmall), unsigned(found.stats.rejectedLarge),
                  unsigned(found.stats.rejectedEdge), unsigned(found.stats.ambiguousPixels));
    if (found.status == sorter::DetectionStatus::Ok) {
      for (size_t i = 0; i < found.count; ++i) {
        const sorter::PixelBlob& b = blobs[i];
        Serial.printf("BLOB class=%u u=%.3f v=%.3f pixels=%u box=%u,%u,%u,%u\n",
                      b.classId, b.centerX, b.centerY, unsigned(b.pixels), b.left,b.top,b.right,b.bottom);
      }
    }
    return;
  }
  Serial.println("ERROR UNKNOWN_COMMAND");
}

void setup() {
  if (CAMERA_PROFILE_CONFIRMED) {
    pinMode(camera_board::FLASH_LED_PIN, OUTPUT);
    digitalWrite(camera_board::FLASH_LED_PIN, LOW);
  }
  Serial.begin(USB_BAUD); Serial.setDebugOutput(false); delay(400);
  Serial.println("SORTER CAMERA NODE; classic ESP32-CAM example; physical calibration required");
  cameraBoot = esp_random(); if (!cameraBoot) cameraBoot = 1;
  camera_board::printHardware();
  cameraReady = camera_board::start(CAMERA_PROFILE_CONFIRMED, FRAMESIZE_QVGA);
  if (cameraReady && !configureSensor()) {
    Serial.println("ERROR SENSOR_CONFIGURATION_FAILED");
    esp_camera_deinit(); cameraReady = false;
  }
  if (cameraReady) {
    workspace.labels = static_cast<uint8_t*>(heap_caps_malloc(PIXELS, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    workspace.queue = static_cast<uint32_t*>(heap_caps_malloc(PIXELS*sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!workspace.labels || !workspace.queue) {
      if (workspace.labels) heap_caps_free(workspace.labels);
      if (workspace.queue) heap_caps_free(workspace.queue);
      workspace.labels = NULL; workspace.queue = NULL;
      Serial.println("ERROR WORKSPACE_ALLOCATION_FAILED");
    } else { workspace.labelCapacity = PIXELS; workspace.queueCapacity = PIXELS; }
    delay(300); // Allow fixed sensor settings to reach the image pipeline.
  }
  calibrationReady = sorter_calibration::CALIBRATION_CONFIRMED && sorter_calibration::CALIBRATION_ID != 0 &&
    sorter_calibration::IMAGE_WIDTH == WIDTH && sorter_calibration::IMAGE_HEIGHT == HEIGHT &&
    mapping.configure(sorter_calibration::HOMOGRAPHY, sorter_calibration::SUPPORT, sorter_calibration::SUPPORT_COUNT);
  if (calibrationReady) {
    for (size_t i = 0; i < sorter_calibration::SUPPORT_COUNT; ++i) {
      const sorter::Point2& p = sorter_calibration::SUPPORT[i];
      if (p.x < 0 || p.y < 0 || p.x > WIDTH-1 || p.y > HEIGHT-1) calibrationReady = false;
    }
  }
  if (CAMERA_PROFILE_CONFIRMED && CAMERA_UART_PINS_CONFIRMED) {
    // Only the reviewed example pair is accepted. SD, flash, boot-strapping,
    // camera, UART0 and PSRAM pins must not be silently substituted here.
    if (CAMERA_UART_RX == 14 && CAMERA_UART_TX == 13) {
      cameraUart.setRxBufferSize(2*sorter::kMaxFrame);
      cameraUart.begin(CAMERA_UART_BAUD, SERIAL_8N1, CAMERA_UART_RX, CAMERA_UART_TX);
      uartReady = true;
    } else Serial.println("ERROR UART_PIN_PROFILE_UNSUPPORTED");
  }
  printStatus();
  Serial.println("Commands: STATUS, STATS, SCAN, CAPTURE, LED ON, LED OFF, HELP");
}

void loop() {
  handleConsole();
  if (uartReady) {
    while (cameraUart.available()) {
      if (wireParser.feed(char(cameraUart.read()), incoming)) handleRequest(incoming);
    }
  }
  delay(1);
}
