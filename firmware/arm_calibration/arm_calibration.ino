#include <Arduino.h>
#include <Protocol.h>
#include <ManualJogCore.h>
#include "LocalConfig.h"

using namespace sorter;
bool inputsReady = false;
volatile bool inputLoss = false;
void ARDUINO_ISR_ATTR inputLostISR() { inputLoss = true; }
bool enablePresent() {
  return inputsReady && !inputLoss && digitalRead(manual::ENABLE_PIN) == HIGH;
}
bool powerPresent() {
  return inputsReady && !inputLoss && digitalRead(manual::POWER_GOOD_PIN) == HIGH;
}
class EspJogIO : public JogIO {
 public:
  bool attached[4] = {false, false, false, false};
  bool enableAt(const JointPose& p) override {
    if (!manual::OUTPUTS_ENABLED || !manual::PINOUT_CONFIRMED || !manual::LIMITS_CONFIRMED ||
        !manual::pinsDistinct()) return false;
    for (uint8_t j = 0; j < 4; ++j) {
      if (!enablePresent() || !powerPresent()) { detachOutputs(); return false; }
      attached[j] = ledcAttach(manual::SERVO_PINS[j], 50, 14);
      if (!attached[j] || !enablePresent() || !powerPresent() ||
          !ledcWrite(manual::SERVO_PINS[j], (uint32_t(p.us[j])*16384U + 10000U)/20000U)) {
        detachOutputs(); return false;
      }
    }
    return true;
  }
  bool writePose(const JointPose& p) override {
    for (uint8_t j = 0; j < 4; ++j)
      if (!attached[j] || !enablePresent() || !powerPresent() ||
          !ledcWrite(manual::SERVO_PINS[j], (uint32_t(p.us[j])*16384U + 10000U)/20000U)) return false;
    return true;
  }
  void detachOutputs() override {
    for (uint8_t j = 0; j < 4; ++j) if (attached[j]) {
      ledcWrite(manual::SERVO_PINS[j], 0);
      ledcDetach(manual::SERVO_PINS[j]);
      pinMode(manual::SERVO_PINS[j], OUTPUT);
      digitalWrite(manual::SERVO_PINS[j], LOW);
      attached[j] = false;
    }
  }
};
JogSettings jogSettings = manual::settings();
EspJogIO io;
ManualJogController controller(jogSettings, io);
char line[100];
uint8_t lineSize = 0;
bool lineInvalid = false;

const char* stateName(JogState s) {
  switch (s) {
    case JogState::Disarmed: return "DISARMED";
    case JogState::Holding: return "HOLDING";
    case JogState::Jogging: return "JOGGING";
    default: return "FAULT";
  }
}
const char* errorName(JogError e) {
  switch (e) {
    case JogError::None: return "NONE";
    case JogError::Config: return "CONFIG_NOT_CONFIRMED";
    case JogError::Seed: return "SEED_REQUIRED_OR_INVALID";
    case JogError::Support: return "SUPPORT_CONFIRM_REQUIRED";
    case JogError::Permit: return "ENABLE_OR_POWER_LOST";
    case JogError::Busy: return "WAIT_FOR_HOLDING";
    case JogError::Range: return "STEP_OR_LIMIT_INVALID";
    case JogError::LeaseExpired: return "HOLD_LEASE_EXPIRED";
    case JogError::SessionExpired: return "SESSION_TIME_LIMIT";
    case JogError::LoopLate: return "CONTROL_LOOP_LATE";
    case JogError::Output: return "PWM_OUTPUT_FAILURE";
    default: return "ESTOP";
  }
}
void status() {
  const JointPose& p = controller.commandedPose();
  Serial.printf("STATUS state=%s error=%s outputs=%u seed=%u permit=%u power=%u command_us=%u,%u,%u,%u\n",
    stateName(controller.state()), errorName(controller.error()), controller.outputsOn(),
    controller.seedPresent(), enablePresent(), powerPresent(), p.us[0], p.us[1], p.us[2], p.us[3]);
}
bool seedFromText(const char* text, JointPose& p) {
  char copy[40];
  if (strlen(text) >= sizeof(copy)) return false;
  strcpy(copy, text); char* fields[4];
  if (!splitExact(copy, ',', fields, 4)) return false;
  for (uint8_t j = 0; j < 4; ++j) {
    uint32_t us;
    if (!parseU32(fields[j], us) || us > UINT16_MAX) return false;
    p.us[j] = uint16_t(us);
  }
  return true;
}
bool jogFromText(const char* text, uint8_t& joint, int32_t& delta) {
  char copy[32];
  if (strlen(text) >= sizeof(copy)) return false;
  strcpy(copy, text); char* fields[2]; uint32_t index;
  if (!splitExact(copy, ' ', fields, 2) || !parseU32(fields[0], index) || index > 3 ||
      !parseI32(fields[1], delta)) return false;
  joint = uint8_t(index); return true;
}
void command(const char* text) {
  if (!strcmp(text, "HELP")) {
    Serial.println("SEED us0,us1,us2,us3 | SUPPORTED | ARM | JOG index delta_us | HOLD | POSE | STATUS | DISARM | ESTOP | RESET");
    Serial.println("Manual commissioning only. Seed can jump on ARM; support the arm. No automatic homing or routes.");
    Serial.println("Wait HOLDING between steps. Max step 25 us, max session 120 s; 15 s inactivity detaches all PWM.");
    return;
  }
  if (!strcmp(text, "STATUS")) { status(); return; }
  if (!strcmp(text, "DISARM")) controller.disarm();
  else if (!strcmp(text, "ESTOP")) controller.estop();
  else if (!strcmp(text, "RESET")) controller.reset();
  else if (!strncmp(text, "SEED ", 5)) {
    JointPose p = {};
    if (!seedFromText(text + 5, p) || !controller.seed(p)) Serial.println("REFUSED SEED");
  } else if (!strcmp(text, "SUPPORTED")) {
    if (!controller.confirmSupported()) Serial.println("REFUSED: first enter SEED while DISARMED");
  } else if (!strcmp(text, "ARM")) {
    // Only an explicit attempt while disarmed may clear an old falling edge.
    if (controller.state() == JogState::Disarmed && inputsReady &&
        digitalRead(manual::ENABLE_PIN) == HIGH && digitalRead(manual::POWER_GOOD_PIN) == HIGH)
      inputLoss = false;
    if (!controller.arm(millis(), enablePresent(), powerPresent())) Serial.println("REFUSED ARM");
  } else if (!strncmp(text, "JOG ", 4)) {
    uint8_t joint = 0; int32_t delta = 0;
    if (!jogFromText(text + 4, joint, delta) ||
        !controller.jog(joint, delta, millis(), enablePresent(), powerPresent())) Serial.println("REFUSED JOG");
  } else if (!strcmp(text, "HOLD")) {
    if (!controller.hold(millis(), enablePresent(), powerPresent())) Serial.println("REFUSED HOLD");
  } else if (!strcmp(text, "POSE")) {
    if (controller.state() != JogState::Holding) Serial.println("REFUSED: pose export requires stationary HOLDING");
    else {
      const JointPose& p = controller.commandedPose();
      Serial.printf("COMMAND_POSE_US,%u,%u,%u,%u\n", p.us[0], p.us[1], p.us[2], p.us[3]);
      Serial.println("Command only: measure actual X/Y/Z and clearance separately. POSE does not renew hold lease.");
    }
  } else Serial.println("Invalid command; HELP");
  controller.tick(millis(), enablePresent(), powerPresent());
  status();
}
void setup() {
  Serial.begin(115200);
  // Apart from the diagnostic Serial console, no GPIO is set up with false defaults.
  if (manual::PINOUT_CONFIRMED && manual::pinsDistinct()) {
    pinMode(manual::ENABLE_PIN, INPUT_PULLDOWN);
    pinMode(manual::POWER_GOOD_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(manual::ENABLE_PIN), inputLostISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(manual::POWER_GOOD_PIN), inputLostISR, FALLING);
    inputsReady = true;
  }
  Serial.println("Assembled-arm manual calibration. Outputs disabled by default. HELP.");
  status();
}
void loop() {
  controller.tick(millis(), enablePresent(), powerPresent());
  for (uint8_t i = 0; i < 64 && Serial.available(); ++i) {
    const char ch = char(Serial.read());
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (!lineInvalid) { line[lineSize] = 0; command(line); }
      else Serial.println("Malformed/oversized command discarded");
      lineSize = 0; lineInvalid = false;
    } else if (!lineInvalid) {
      if (ch < 32 || ch > 126 || lineSize + 1 >= sizeof(line)) lineInvalid = true;
      else line[lineSize++] = ch;
    }
  }
  static JogState reported = JogState::Disarmed;
  if (reported != controller.state()) { reported = controller.state(); status(); }
  delay(1);
}
