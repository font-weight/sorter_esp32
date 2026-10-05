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
volatile bool enableLossLatched = false, supplyLossLatched = false;
void ARDUINO_ISR_ATTR enableLostISR() { enableLossLatched = true; }
void ARDUINO_ISR_ATTR supplyLostISR() { supplyLossLatched = true; }
bool hardwareEnable();
bool supplyGood();

class EspMotionIO : public MotionIO {
 public:
  bool attached[4] = {false, false, false, false};
  bool hardwareEnablePresent() const override { return hardwareEnable(); }
  bool servoSupplyPresent() const override { return supplyGood(); }
  bool enableAt(const JointPose& pose) override {
    if (local::VIRTUAL_MODE) return true;
    if (!local::MOTOR_OUTPUTS_ENABLED || !local::PINOUT_CONFIRMED ||
        !local::pinsDistinct() || !hardwareInputsReady || !uartReady) return false;
    for (uint8_t j = 0; j < 4; ++j) {
      if (!hardwareEnable() || !supplyGood()) { detachOutputs(); return false; }
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
      if (!hardwareEnable() || !supplyGood() || !attached[j] || !ledcWrite(local::SERVO_PINS[j],
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

bool hardwareEnable() {
  return local::VIRTUAL_MODE ? simulatedEnable :
      hardwareInputsReady && !enableLossLatched && digitalRead(local::HARDWARE_ENABLE_PIN) == HIGH;
}
bool supplyGood() {
  return local::VIRTUAL_MODE ? simulatedSupply :
      hardwareInputsReady && !supplyLossLatched && digitalRead(local::SERVO_POWER_GOOD_PIN) == HIGH;
}
void printStatus() {
  const JointPose& p = controller.commandedPose();
  Serial.printf("STATUS mode=%s state=%s phase=%s error=%s outputs=%u attempts=%u cleared=%u "
                "session=%lu seq=%lu stale=%lu pwm=%u,%u,%u,%u\n",
    local::VIRTUAL_MODE ? "VIRTUAL_NO_PWM" : "HARDWARE", motionStateName(controller.state()),
    controller.phaseName(), motionErrorName(controller.error()), controller.outputsOn(),
    controller.attempts(), controller.clearedTargets(), (unsigned long)controller.session(),
    (unsigned long)controller.sequence(), (unsigned long)controller.staleReplies(),
    p.us[0], p.us[1], p.us[2], p.us[3]);
}
void handlePacket(const Packet& p) {
  if (p.type == 'D') {
    Scene scene = {};
    if (decodeScene(p.payload, scene)) controller.receiveScene(p.session, p.seq, scene, millis());
    else Serial.println("RX_BAD_SCENE (ignored; current request will time out)");
  } else if (p.type == 'E') {
    Serial.printf("CAMERA_ERROR session=%lu seq=%lu code=%s\n",
                  (unsigned long)p.session, (unsigned long)p.seq, p.payload);
    controller.receiveError(p.session, p.seq);
  }
}
void command(const char* line) {
  if (!strcmp(line, "STATUS")) { printStatus(); return; }
  if (!strcmp(line, "HELP")) {
    Serial.println("STATUS | PARKCONFIRM | ARM | PAUSE | RESUME | DISARM | ESTOP | RESET");
    Serial.println("Virtual: SCENE <D-payload> | FRAME <full-frame> | SIMPOWER 0/1 | SIMENABLE 0/1");
    return;
  }
  if (!strcmp(line, "PARKCONFIRM")) {
    controller.confirmOperatorPark(); Serial.println("PARK acknowledgment recorded only when disarmed/done.");
  } else if (!strcmp(line, "ARM")) {
    // A new explicit ARM can clear a past falling edge only while de-energized.
    if (!local::VIRTUAL_MODE && hardwareInputsReady &&
        (controller.state() == MotionState::Disarmed || controller.state() == MotionState::Done) &&
        digitalRead(local::HARDWARE_ENABLE_PIN) == HIGH && digitalRead(local::SERVO_POWER_GOOD_PIN) == HIGH) {
      enableLossLatched = false; supplyLossLatched = false;
    }
    uint32_t session = esp_random(); if (!session) session = 1;
    controller.arm(millis(), session, hardwareEnable(), supplyGood());
  } else if (!strcmp(line, "PAUSE")) controller.pause(millis());
  else if (!strcmp(line, "RESUME")) controller.resume(millis());
  else if (!strcmp(line, "DISARM")) controller.disarm();
  else if (!strcmp(line, "ESTOP")) controller.emergencyStop();
  else if (!strcmp(line, "RESET")) controller.resetFault();
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
  controller.tick(millis(), hardwareEnable(), supplyGood());
  printStatus();
}
void setup() {
  Serial.begin(115200);
  // No servo GPIO or peripheral is configured during default virtual startup.
  if (!local::VIRTUAL_MODE && local::PINOUT_CONFIRMED && local::pinsDistinct()) {
    pinMode(local::HARDWARE_ENABLE_PIN, INPUT_PULLDOWN);
    pinMode(local::SERVO_POWER_GOOD_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(local::HARDWARE_ENABLE_PIN), enableLostISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(local::SERVO_POWER_GOOD_PIN), supplyLostISR, FALLING);
    hardwareInputsReady = true;
  }
  if (local::UART_PROFILE_CONFIRMED && local::pinsDistinct()) {
    cameraLink.begin(local::UART_BAUD, SERIAL_8N1, local::UART_RX_PIN, local::UART_TX_PIN);
    uartReady = true;
  }
  Serial.println("Sorter motion prototype. No physical calibration supplied. HELP for commands.");
  printStatus();
}
void loop() {
  controller.tick(millis(), hardwareEnable(), supplyGood());
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
  }
  delay(1);
}
