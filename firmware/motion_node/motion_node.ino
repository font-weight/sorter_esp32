#include <Arduino.h>
#include <esp_system.h>
#include <MotionCore.h>
#include "LocalConfig.h"

using namespace sorter;
HardwareSerial cameraLink(1);
MotionCalibration calibration = local::makeCalibration();
MotionSettings settings = local::makeSettings();
bool uartReady = false;
bool hardwareInputsReady = false;
bool simulatedEnable = true, simulatedSupply = true;
volatile bool enableLossLatched = false;
void ARDUINO_ISR_ATTR enableLostISR() { enableLossLatched = true; }
bool hardwareEnable();
bool servoSupplyAllowed();

class EspMotionIO : public MotionIO {
 public:
  bool attached[4] = {false, false, false, false};
  bool hardwareEnablePresent() const override { return hardwareEnable(); }
  bool servoSupplyPresent() const override { return servoSupplyAllowed(); }
  bool enableAt(const JointPose& pose) override {
    if (local::VIRTUAL_MODE) return true;
    if (!local::MOTOR_OUTPUTS_ENABLED || !local::PINOUT_CONFIRMED ||
        !local::pinsDistinct() || !hardwareInputsReady || !uartReady) return false;
    for (uint8_t j = 0; j < 4; ++j) {
      if (!hardwareEnable() || !servoSupplyAllowed()) { detachOutputs(); return false; }
      attached[j] = ledcAttach(local::SERVO_PINS[j], 50, 14);
      if (!attached[j]) { detachOutputs(); return false; }
      // 14-bit timer counts 16384 ticks per 20 ms period.
      if (!ledcWrite(local::SERVO_PINS[j], (uint32_t(pose.us[j])*16384U + 10000U)/20000U)) {
        detachOutputs(); return false;
      }
    }
    return true;
  }
  bool writePose(const JointPose& pose) override {
    if (local::VIRTUAL_MODE) return true;
    if (!local::MOTOR_OUTPUTS_ENABLED || !local::PINOUT_CONFIRMED) return false;
    for (uint8_t j = 0; j < 4; ++j)
      if (!hardwareEnable() || !servoSupplyAllowed() || !attached[j] || !ledcWrite(local::SERVO_PINS[j],
          (uint32_t(pose.us[j])*16384U + 10000U)/20000U)) return false;
    return true;
  }
  void detachOutputs() override {
    if (local::VIRTUAL_MODE) return;
    for (uint8_t j = 0; j < 4; ++j) if (attached[j]) {
      // A software detach is not an emergency power disconnect.
      ledcWrite(local::SERVO_PINS[j], 0);
      ledcDetach(local::SERVO_PINS[j]);
      pinMode(local::SERVO_PINS[j], OUTPUT);
      digitalWrite(local::SERVO_PINS[j], LOW);
      attached[j] = false;
    }
  }
  bool requestScene(uint32_t session, uint32_t seq, uint32_t calibrationId) override {
    Packet p = {}; p.type = 'Q'; p.session = session; p.seq = seq;
    snprintf(p.payload, sizeof(p.payload), "%lu", (unsigned long)calibrationId);
    char frame[kMaxFrame];
    if (!encodePacket(p, frame, sizeof(frame))) return false;
    Serial.print("Q_FRAME "); Serial.print(frame);
    if (uartReady) return cameraLink.write(reinterpret_cast<const uint8_t*>(frame), strlen(frame)) == strlen(frame);
    return local::VIRTUAL_MODE;
  }
};
EspMotionIO motionIO;
MotionController controller(calibration, settings, motionIO);
LineParser linkParser;
char consoleLine[kMaxFrame + 16];
size_t consoleLength = 0;
bool consoleOverflow = false;
// One read-only camera transaction, separate from the motion controller.
struct CameraScan {
  bool pending = false;
  uint32_t session = 0, seq = 0, calibrationId = 0, started = 0;
};
CameraScan cameraScan;
constexpr uint32_t CAMERA_SCAN_TIMEOUT_MS = 6000;

void expireCameraScan(uint32_t now) {
  if (cameraScan.pending && uint32_t(now-cameraScan.started) >= CAMERA_SCAN_TIMEOUT_MS) {
    cameraScan.pending = false;
    Serial.println("CAMSCAN_TIMEOUT: no valid camera reply within 6000 ms; send CAMSCAN to retry.");
  }
}
void cancelCameraScan() {
  if (cameraScan.pending) Serial.println("CAMSCAN_CANCELLED");
  cameraScan.pending = false;
}
void startCameraScan() {
  const uint32_t now = millis();
  expireCameraScan(now);
  if (!local::VIRTUAL_MODE) { Serial.println("CAMSCAN_REFUSED reason=HARDWARE_MODE; use VIRTUAL_MODE=true"); return; }
  if (cameraScan.pending) { Serial.println("CAMSCAN_REFUSED reason=BUSY; wait for the current reply"); return; }
  if (controller.outputsOn() ||
      (controller.state() != MotionState::Disarmed && controller.state() != MotionState::Done)) {
    Serial.println("CAMSCAN_REFUSED reason=MOTION_ACTIVE; DISARM first"); return;
  }
  if (!uartReady) { Serial.println("CAMSCAN_REFUSED reason=UART_DISABLED; confirm UART profile and upload motion_node"); return; }
  if (!cameraScan.session || cameraScan.seq == UINT32_MAX) {
    const uint32_t previous = cameraScan.session;
    cameraScan.session = esp_random();
    // One random draw and at most four increments; repeated RNG values cannot block.
    for (uint8_t n = 0; n < 4 && (!cameraScan.session || cameraScan.session == previous ||
         cameraScan.session == controller.session()); ++n) ++cameraScan.session;
    cameraScan.seq = 0;
  }
  ++cameraScan.seq; // A new sequence requests a fresh frame, not the camera cache.
  cameraScan.calibrationId = calibration.calibrationId;
  cameraScan.started = now; cameraScan.pending = true;
  if (!motionIO.requestScene(cameraScan.session, cameraScan.seq, cameraScan.calibrationId)) {
    cameraScan.pending = false;
    Serial.println("CAMSCAN_ERROR code=UART_WRITE_FAILED"); return;
  }
  Serial.printf("CAMSCAN_WAIT session=%lu seq=%lu timeout_ms=%lu; VIRTUAL_NO_PWM\n",
    (unsigned long)cameraScan.session, (unsigned long)cameraScan.seq,
    (unsigned long)CAMERA_SCAN_TIMEOUT_MS);
}

bool hardwareEnable() {
  return local::VIRTUAL_MODE ? simulatedEnable :
      hardwareInputsReady && !enableLossLatched && digitalRead(local::HARDWARE_ENABLE_PIN) == HIGH;
}
bool servoSupplyAllowed() {
  // Hardware mode does not measure the servo supply. Keep the core's simulated
  // supply input for virtual fault tests; physical permission uses ENABLE only.
  return local::VIRTUAL_MODE ? simulatedSupply : hardwareInputsReady;
}
void printStatus() {
  const JointPose& p = controller.commandedPose();
  const bool physicalPwm = motionIO.attached[0] || motionIO.attached[1] ||
                           motionIO.attached[2] || motionIO.attached[3];
  Serial.printf("STATUS mode=%s state=%s phase=%s error=%s outputs=%u physical_pwm=%u attempts=%u cleared=%u "
                "session=%lu seq=%lu stale=%lu pwm=%u,%u,%u,%u power_monitor=%s uart=%u camscan_pending=%u batch_limit_ms=%lu\n",
    local::VIRTUAL_MODE ? "VIRTUAL_NO_PWM" : "HARDWARE", motionStateName(controller.state()),
    controller.phaseName(), motionErrorName(controller.error()), controller.outputsOn(), physicalPwm,
    controller.attempts(), controller.clearedTargets(), (unsigned long)controller.session(),
    (unsigned long)controller.sequence(), (unsigned long)controller.staleReplies(),
    p.us[0], p.us[1], p.us[2], p.us[3], local::VIRTUAL_MODE ? "SIMULATED" : "NONE",
    uartReady, cameraScan.pending, (unsigned long)settings.maxBatchMs);
}
void handlePacket(const Packet& p) {
  expireCameraScan(millis());
  if ((p.type == 'D' || p.type == 'E') && cameraScan.session && p.session == cameraScan.session) {
    // Diagnostic replies never enter MotionController, including duplicates,
    // cancelled replies and earlier sequences from this diagnostic session.
    if (!cameraScan.pending || p.seq != cameraScan.seq) return;
    if (p.type == 'E') {
      cameraScan.pending = false;
      Serial.printf("CAMSCAN_ERROR code=%s\n", p.payload); return;
    }
    Scene scene = {};
    if (!decodeScene(p.payload, scene)) { Serial.println("CAMSCAN_BAD_SCENE ignored"); return; }
    if (scene.calibrationId != cameraScan.calibrationId) {
      cameraScan.pending = false;
      Serial.printf("CAMSCAN_ERROR code=CALIBRATION_MISMATCH expected=%lu received=%lu\n",
        (unsigned long)cameraScan.calibrationId, (unsigned long)scene.calibrationId); return;
    }
    cameraScan.pending = false;
    Serial.printf("CAMSCENE count=%u calibrationId=%lu boot=%lu\n", scene.count,
      (unsigned long)scene.calibrationId, (unsigned long)scene.cameraBoot);
    for (size_t i = 0; i < scene.count; ++i) {
      const Detection& d = scene.objects[i];
      Serial.printf("OBJECT class=%u color=%s x_mm=%.1f y_mm=%.1f pixels=%lu in_pick_area=%u\n",
        d.classId, d.classId == kRedClassId ? "red" : "blue", d.x10/10.0, d.y10/10.0,
        (unsigned long)d.pixels, inPickArea(d.x10,d.y10,calibration));
    }
    return;
  }
  if (p.type == 'D') {
    Scene scene = {};
    if (decodeScene(p.payload, scene)) {
      if (controller.receiveScene(p.session, p.seq, scene, millis())) {
        Serial.printf("LIVE_SCENE count=%u calibrationId=%lu boot=%lu\n", scene.count,
          (unsigned long)scene.calibrationId, (unsigned long)scene.cameraBoot);
        if (controller.state() == MotionState::Moving) {
          const Detection& target = controller.selectedTarget();
          Serial.printf("SORT_TARGET class=%u color=%s bin=%u x_mm=%.1f y_mm=%.1f pixels=%lu\n",
            target.classId, target.classId == kRedClassId ? "red" : "blue",
            unsigned(classBinIndex(target.classId)) + 1, target.x10/10.0, target.y10/10.0,
            (unsigned long)target.pixels);
        }
      }
    }
    else Serial.println("RX_BAD_SCENE (ignored; current request will time out)");
  } else if (p.type == 'E') {
    Serial.printf("CAMERA_ERROR session=%lu seq=%lu code=%s\n",
                  (unsigned long)p.session, (unsigned long)p.seq, p.payload);
    controller.receiveError(p.session, p.seq);
  }
}
void command(const char* line) {
  expireCameraScan(millis());
  if (!strcmp(line, "STATUS")) { printStatus(); return; }
  if (!strcmp(line, "CAMSCAN")) { startCameraScan(); return; }
  if (!strcmp(line, "HELP")) {
    Serial.println("STATUS | PARKCONFIRM | ARM | PAUSE | RESUME | DISARM | ESTOP | RESET");
    Serial.println("Virtual: SCENE <D-payload> | FRAME <full-frame> | SIMPOWER 0/1 | SIMENABLE 0/1");
    Serial.println("CAMSCAN: one real-camera UART scan while VIRTUAL_NO_PWM and DISARMED/DONE; no ARM required.");
    Serial.println("Expect CAMSCENE count and OBJECT red/blue x_mm/y_mm; 6000 ms timeout, no automatic retry.");
    Serial.println("Virtual live cycle: PARKCONFIRM then ARM; real camera replies drive the simulated route.");
    Serial.println("At SIM_REMOVE remove only the selected part from camera view; physical_pwm stays 0.");
    Serial.println("Hardware: servo supply is not monitored. Disable ENABLE before changing servo power.");
    return;
  }
  if (!strcmp(line, "PARKCONFIRM")) {
    controller.confirmOperatorPark(); Serial.println("PARK acknowledgment recorded only when disarmed/done.");
  } else if (!strcmp(line, "ARM")) {
    if (cameraScan.pending) { Serial.println("ARM_REFUSED reason=CAMSCAN_BUSY; wait or DISARM to cancel"); return; }
    // A new explicit ARM can clear a past ENABLE edge only while disarmed/done.
    if (!local::VIRTUAL_MODE && hardwareInputsReady &&
        (controller.state() == MotionState::Disarmed || controller.state() == MotionState::Done) &&
        digitalRead(local::HARDWARE_ENABLE_PIN) == HIGH) {
      enableLossLatched = false;
    }
    uint32_t session = esp_random(); if (!session) session = 1;
    if (session == cameraScan.session) { ++session; if (!session) session = 1; }
    controller.arm(millis(), session, hardwareEnable(), servoSupplyAllowed());
  } else if (!strcmp(line, "PAUSE")) controller.pause(millis());
  else if (!strcmp(line, "RESUME")) controller.resume(millis());
  else if (!strcmp(line, "DISARM")) { cancelCameraScan(); controller.disarm(); }
  else if (!strcmp(line, "ESTOP")) { cancelCameraScan(); controller.emergencyStop(); }
  else if (!strcmp(line, "RESET")) { cancelCameraScan(); controller.resetFault(); }
  else if (local::VIRTUAL_MODE && !strncmp(line, "SCENE ", 6)) {
    Scene scene = {};
    if (!decodeScene(line + 6, scene)) Serial.println("SCENE syntax invalid");
    else controller.receiveScene(controller.session(), controller.sequence(), scene, millis());
  } else if (local::VIRTUAL_MODE && !strncmp(line, "FRAME ", 6)) {
    Packet p = {};
    if (decodePacket(line + 6, p)) handlePacket(p); else Serial.println("FRAME invalid");
  } else if (local::VIRTUAL_MODE && !strcmp(line, "SIMPOWER 0")) simulatedSupply = false;
  else if (local::VIRTUAL_MODE && !strcmp(line, "SIMPOWER 1")) simulatedSupply = true;
  else if (local::VIRTUAL_MODE && !strcmp(line, "SIMENABLE 0")) simulatedEnable = false;
  else if (local::VIRTUAL_MODE && !strcmp(line, "SIMENABLE 1")) simulatedEnable = true;
  else Serial.println("Unknown/restricted command; HELP");
  // Ensure a physical input change is handled before any subsequent local command.
  controller.tick(millis(), hardwareEnable(), servoSupplyAllowed());
  printStatus();
}
void setup() {
  Serial.begin(115200);
  // No servo GPIO or peripheral is configured during default virtual startup.
  if (!local::VIRTUAL_MODE && local::PINOUT_CONFIRMED && local::pinsDistinct()) {
    for (uint8_t j = 0; j < 4; ++j) {
      pinMode(local::SERVO_PINS[j], OUTPUT);
      digitalWrite(local::SERVO_PINS[j], LOW);
    }
    pinMode(local::HARDWARE_ENABLE_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(local::HARDWARE_ENABLE_PIN), enableLostISR, FALLING);
    hardwareInputsReady = true;
  }
  if (local::UART_PROFILE_CONFIRMED && local::pinsDistinct()) {
    cameraLink.begin(local::UART_BAUD, SERIAL_8N1, local::UART_RX_PIN, local::UART_TX_PIN);
    uartReady = true;
  }
  Serial.println("Sorter motion. Measured pose map loaded; physical routes still require verification. HELP for commands.");
  if (local::VIRTUAL_MODE && uartReady)
    Serial.println("Virtual live-camera cycle ready: PARKCONFIRM, ARM; follow SORT_TARGET and SIM_REMOVE.");
  printStatus();
}
void loop() {
  controller.tick(millis(), hardwareEnable(), servoSupplyAllowed());
  expireCameraScan(millis());
  // Bound each serial service pass so an input flood cannot starve safety checks.
  if (uartReady) for (uint16_t n = 0; n < 128 && cameraLink.available(); ++n) {
    Packet p = {};
    if (linkParser.feed(char(cameraLink.read()), p)) handlePacket(p);
  }
  for (uint16_t n = 0; n < 128 && Serial.available(); ++n) {
    const char ch = char(Serial.read());
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (!consoleOverflow) { consoleLine[consoleLength] = 0; command(consoleLine); }
      else Serial.println("Command too long; discarded");
      consoleLength = 0; consoleOverflow = false;
    } else if (!consoleOverflow) {
      if (consoleLength + 1 < sizeof(consoleLine)) consoleLine[consoleLength++] = ch;
      else consoleOverflow = true;
    }
  }
  static MotionState reportedState = MotionState::Disarmed;
  static const char* reportedPhase = "";
  if (reportedState != controller.state() || strcmp(reportedPhase, controller.phaseName())) {
    reportedState = controller.state(); reportedPhase = controller.phaseName(); printStatus();
    if (local::VIRTUAL_MODE && reportedState == MotionState::Moving && !strcmp(reportedPhase, "RELEASE")) {
      const Detection& target = controller.selectedTarget();
      Serial.printf("SIM_REMOVE color=%s x_mm=%.1f y_mm=%.1f: remove this part from camera view before the next scan.\n",
        target.classId == kRedClassId ? "red" : "blue", target.x10/10.0, target.y10/10.0);
    }
  }
  delay(1);
}
