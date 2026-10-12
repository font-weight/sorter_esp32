#include <Arduino.h>
#include <Protocol.h>
#include <ManualJogCore.h>
#include "LocalConfig.h"
#include "RedPickTrial.h"

using namespace sorter;
bool inputsReady = false;
volatile bool enableLossLatched = false;
void ARDUINO_ISR_ATTR enableLostISR() { enableLossLatched = true; }
bool enablePresent() {
  return inputsReady && !enableLossLatched && digitalRead(manual::ENABLE_PIN) == HIGH;
}
class EspJogIO : public JogIO {
 public:
  bool attached[4] = {false, false, false, false};
  bool enableAt(const JointPose& p) override {
    if (!manual::OUTPUTS_ENABLED || !manual::PINOUT_CONFIRMED || !manual::LIMITS_CONFIRMED ||
        !manual::pinsDistinct()) return false;
    for (uint8_t j = 0; j < 4; ++j) {
      if (!enablePresent()) { detachOutputs(); return false; }
      attached[j] = ledcAttach(manual::SERVO_PINS[j], 50, 14);
      if (!attached[j] || !enablePresent() ||
          !ledcWrite(manual::SERVO_PINS[j], (uint32_t(p.us[j])*16384U + 10000U)/20000U)) {
        detachOutputs(); return false;
      }
    }
    return true;
  }
  bool writePose(const JointPose& p) override {
    for (uint8_t j = 0; j < 4; ++j)
      if (!attached[j] || !enablePresent() ||
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
HardwareSerial trialCamera(1);
bool trialUartReady = false;
LineParser trialParser;
red_trial::RedPickTrial redTrial(controller, jogSettings, trialCamera);
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
    case JogError::Permit: return "ENABLE_LOST";
    case JogError::Busy: return "WAIT_FOR_HOLDING";
    case JogError::Range: return "STEP_OR_LIMIT_INVALID";
    case JogError::LeaseExpired: return "HOLD_LEASE_EXPIRED";
    case JogError::SessionExpired: return "SESSION_TIME_LIMIT";
    case JogError::LoopLate: return "CONTROL_LOOP_LATE";
    case JogError::Output: return "PWM_OUTPUT_FAILURE";
    default: return "ESTOP";
  }
}
void printPower() {
  Serial.println("POWER monitor=NONE voltage_feedback=UNAVAILABLE");
}
void status() {
  const JointPose& p = controller.commandedPose();
  const uint32_t now = millis();
  Serial.printf("STATUS state=%s error=%s outputs=%u seed=%u permit=%u power_monitor=NONE command_us=%u,%u,%u,%u idle_left_s=%lu session_left_s=%lu camera_uart=%u trial=%s trial_error=%s camera_action=%s sorted_red=%u sorted_blue=%u sort_attempts=%u\n",
    stateName(controller.state()), errorName(controller.error()), controller.outputsOn(),
    controller.seedPresent(), enablePresent(), p.us[0], p.us[1], p.us[2], p.us[3],
    (unsigned long)((controller.holdRemainingMs(now) + 999U) / 1000U),
    (unsigned long)((controller.sessionRemainingMs(now) + 999U) / 1000U),
    trialUartReady, redTrial.stateName(), redTrial.error(), redTrial.actionName(), redTrial.sortedRed(), redTrial.sortedBlue(), redTrial.sortAttempts());
}
void renewInspectionLease() {
  // Only explicit operator commands renew inactivity; automatic reports do not.
  if (controller.state() == JogState::Holding)
    controller.hold(millis(), enablePresent(), true);
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
  if (!strcmp(text, "TRIALRED") || !strcmp(text, "AIMRED") || !strcmp(text, "SORT") || !strcmp(text, "CHECKPICK")) {
    // All camera actions share one busy state and cannot overlap.
    redTrial.start(millis(), trialUartReady, enablePresent(), !strcmp(text, "AIMRED"), !strcmp(text, "SORT"), !strcmp(text, "CHECKPICK")); status(); return;
  }
  if (!strcmp(text, "NEXT") || !strcmp(text, "BACK")) {
    redTrial.checkCommand(!strcmp(text, "BACK"), millis(), enablePresent()); status(); return;
  }
  if (redTrial.busy() && strcmp(text, "STATUS") && strcmp(text, "POSE") &&
      strcmp(text, "HOLD") && strcmp(text, "HELP") && strcmp(text, "POWER") &&
      strcmp(text, "DISARM") && strcmp(text, "ESTOP") && strcmp(text, "RESET")) {
    Serial.println("REFUSED: camera action active; DISARM or ESTOP to stop, no manual MOVE/JOG or other camera scans.");
    status(); return;
  }
  if (!strcmp(text, "HELP")) {
    renewInspectionLease();
    Serial.println("SEED us0,us1,us2,us3 | SUPPORTED | ARM | JOG index delta_us | MOVE us0,us1,us2,us3 | HOLD | POSE | STATUS | POWER | DISARM | ESTOP | RESET");
    Serial.println("Channels: 0=base, 1=shoulder, 2=second link, 3=gripper. Delta is microseconds, not degrees.");
    Serial.printf("Max step %u us; rate %u us/s; idle %lu s; session %lu s from ARM. Wait HOLDING between moves.\n",
                  jogSettings.maxStepUs, jogSettings.rateUsPerSecond,
                  (unsigned long)(jogSettings.holdLeaseMs / 1000U),
                  (unsigned long)(jogSettings.maxSessionMs / 1000U));
    for (uint8_t j = 0; j < 4; ++j)
      Serial.printf("LIMIT channel=%u min_us=%u max_us=%u\n", j, jogSettings.minUs[j], jogSettings.maxUs[j]);
    Serial.println("MOVE uses absolute pulse widths in microseconds within every LIMIT, from the current command.");
    Serial.println("All four channels ramp with common progress at the configured rate; delays do not cause catch-up jumps.");
    Serial.println("JOG keeps its Max step limit; MOVE is not limited by that step. No queue: wait HOLDING before another move.");
    Serial.println("MOVE is pulse interpolation, not a straight Cartesian path or homing. Inspect the whole path.");
    Serial.println("STATUS, HELP, POWER, POSE and HOLD renew idle time while HOLDING. Total session does not restart.");
    Serial.println("SEED sets the initial command; SUPPORTED confirms support; ARM starts holding all four channels.");
    Serial.println("Seed can jump on ARM. Use 1500 only after removing all mechanical stops. No automatic homing.");
    Serial.println("No servo supply monitoring. DISARM before every motor-power off/on; ARM may jump to SEED.");
    Serial.println("Enable switch is a software input. Use a separate physical motor-power disconnect.");
    Serial.println("TRIALRED: one real red transfer from fixed 20,80 mm; start in PARK 1500,1200,1500,2100.");
    Serial.println("Camera must see exactly one red part: X=5..35 mm, Y=65..95 mm (inclusive +/-15 mm per axis).");
    Serial.println("Pick route stays at 20,80 mm; offset grip success is not yet verified. Use the same part and orientation.");
    Serial.println("Both camera frames are captured in PARK; ends HOLDING in PARK. No grid interpolation or automatic retry.");
    Serial.println("AIMRED: from the same PARK, one fresh red at X=20..130, Y=25..80 mm; HUB then upper pose from camera coordinates.");
    Serial.println("AIMRED stops HOLDING with the gripper open near Z=100 mm; no descent, grip or automatic return. Inspect alignment.");
    Serial.println("Return with MOVE 1500,1500,1500,2100 then MOVE 1500,1200,1500,2100; wait HOLDING after each.");
    Serial.println("SORT: finite real batch of red/blue parts, X=20..130 Y=25..80 mm; start HOLDING in PARK.");
    Serial.printf("Side entry: lower at (X,Y-%ld), then enter +Y on lower map before closing. Target Y must be >= %ld mm. No boundary clamping.\n",
      (long)manual::SIDE_APPROACH_MM, long(25+manual::SIDE_APPROACH_MM));
    Serial.println("SIDE_APPROACH_OUTSIDE_AREA or SIDE_PATH_BLOCKED refuses before this transfer; keep the approach corridor clear.");
    Serial.println("Offset is centre-to-centre distance, without adding half the part size. Observe descent clearance.");
    Serial.println("Up to 8 parts; centres at least 30 mm apart; use 40 mm spacing for initial batches. No parts added or moved by hand.");
    Serial.println("Fresh image after every PARK return; exactly one selected-colour part must disappear. No automatic re-grab on failure.");
    Serial.println("Batch limit 15 min; camera reply 6 s. Ends HOLDING in PARK; support then DISARM before power off.");
    Serial.println("CHECKPICK: one red or blue part; PARK -> HUB -> SIDE_HIGH -> PAUSED, gripper stays open.");
    Serial.println("Mark the part centre. At SIDE_HIGH remove it while stationary, then NEXT -> SIDE_LOW, NEXT -> TARGET_LOW; each stops PAUSED.");
    Serial.println("Measure actual XYZ at each CHECKPOINT. NEXT/BACK are accepted only while PAUSED; no command queue.");
    Serial.println("BACK retraces reached waypoints then HUB/PARK; no grip, bin transfer or second camera scan.");
    Serial.println("CHECKPICK limit 15 min; idle 5 min (POSE/HOLD renew); ARM session 30 min. Use ESTOP for unexpected contact.");
    return;
  }
  if (!strcmp(text, "STATUS")) { renewInspectionLease(); status(); return; }
  if (!strcmp(text, "POWER")) { renewInspectionLease(); printPower(); return; }
  if (!strcmp(text, "DISARM")) { redTrial.cancel("DISARM"); controller.disarm(); }
  else if (!strcmp(text, "ESTOP")) { redTrial.cancel("ESTOP"); controller.estop(); }
  else if (!strcmp(text, "RESET")) { redTrial.cancel("RESET"); controller.reset(); }
  else if (!strncmp(text, "SEED ", 5)) {
    JointPose p = {};
    if (!seedFromText(text + 5, p) || !controller.seed(p)) Serial.println("REFUSED SEED");
  } else if (!strcmp(text, "SUPPORTED")) {
    if (!controller.confirmSupported()) Serial.println("REFUSED: first enter SEED while DISARMED");
  } else if (!strcmp(text, "ARM")) {
    // Only an explicit attempt while disarmed may clear an old falling edge.
    if (controller.state() == JogState::Disarmed && inputsReady &&
        digitalRead(manual::ENABLE_PIN) == HIGH)
      enableLossLatched = false;
    // The core's supply argument is unused in this profile; true is not a measurement.
    if (!controller.arm(millis(), enablePresent(), true)) Serial.println("REFUSED ARM");
  } else if (!strncmp(text, "JOG ", 4)) {
    uint8_t joint = 0; int32_t delta = 0;
    if (!jogFromText(text + 4, joint, delta))
      Serial.println("REFUSED JOG: use JOG channel delta_us; channel=0..3");
    else {
      const uint16_t from = controller.commandedPose().us[joint];
      if (controller.jog(joint, delta, millis(), enablePresent(), true))
        Serial.printf("MOVE channel=%u from_us=%u target_us=%u delta_us=%ld; wait for HOLDING\n",
          joint, from, uint16_t(int32_t(from) + delta), (long)delta);
      else if (controller.error() == JogError::Range) {
        if (!delta || delta < -int32_t(jogSettings.maxStepUs) || delta > jogSettings.maxStepUs)
          Serial.printf("REFUSED JOG: use nonzero delta from -%u to %u us\n", jogSettings.maxStepUs, jogSettings.maxStepUs);
        else
          Serial.printf("REFUSED JOG: target_us=%ld outside channel=%u range=%u..%u; position unchanged\n",
            (long)(int32_t(from) + delta), joint, jogSettings.minUs[joint], jogSettings.maxUs[joint]);
      } else if (controller.error() == JogError::Busy)
        Serial.println("REFUSED JOG: previous move is running; wait for state=HOLDING");
      else if (!controller.outputsOn())
        Serial.println("REFUSED JOG: outputs are off; check STATUS, then SEED, SUPPORTED, ARM");
      else Serial.printf("REFUSED JOG: %s\n", errorName(controller.error()));
    }
  } else if (!strncmp(text, "MOVE ", 5)) {
    JointPose target = {};
    if (!seedFromText(text + 5, target))
      Serial.println("REFUSED MOVE: use MOVE us0,us1,us2,us3; absolute microseconds");
    else {
      const JointPose from = controller.commandedPose();
      // The core's supply argument is unused in this profile; true is not a measurement.
      if (controller.movePose(target, millis(), enablePresent(), true))
        Serial.printf("MOVE_POSE from_us=%u,%u,%u,%u target_us=%u,%u,%u,%u; wait for HOLDING\n",
          from.us[0], from.us[1], from.us[2], from.us[3],
          target.us[0], target.us[1], target.us[2], target.us[3]);
      else if (controller.error() == JogError::Range) {
        for (uint8_t j = 0; j < 4; ++j)
          if (target.us[j] < jogSettings.minUs[j] || target.us[j] > jogSettings.maxUs[j]) {
            Serial.printf("REFUSED MOVE: target_us=%u outside channel=%u range=%u..%u; target unchanged\n",
              target.us[j], j, jogSettings.minUs[j], jogSettings.maxUs[j]);
            break;
          }
      } else if (controller.error() == JogError::Busy)
        Serial.println("REFUSED MOVE: previous move is running; wait for state=HOLDING; no queue");
      else if (!controller.outputsOn())
        Serial.println("REFUSED MOVE: outputs are off; check STATUS, then SEED, SUPPORTED, ARM");
      else Serial.printf("REFUSED MOVE: %s\n", errorName(controller.error()));
    }
  } else if (!strcmp(text, "HOLD")) {
    if (!controller.hold(millis(), enablePresent(), true)) Serial.println("REFUSED HOLD");
  } else if (!strcmp(text, "POSE")) {
    renewInspectionLease();
    if (controller.state() != JogState::Holding) Serial.println("REFUSED: pose export requires stationary HOLDING");
    else {
      const JointPose& p = controller.commandedPose();
      Serial.printf("COMMAND_POSE_US,%u,%u,%u,%u\n", p.us[0], p.us[1], p.us[2], p.us[3]);
      Serial.println("Command only, not measured angles. POSE renewed idle time; measure actual X/Y/Z separately.");
    }
  } else Serial.println("Invalid command; HELP");
  controller.tick(millis(), enablePresent(), true);
  status();
}
void setup() {
  Serial.begin(115200);
  // Servo pins stay idle; no PWM until explicit SEED, SUPPORTED and ARM.
  if (manual::PINOUT_CONFIRMED && manual::pinsDistinct()) {
    // Defined idle signal; no PWM until explicit ARM.
    for (uint8_t j = 0; j < 4; ++j) {
      pinMode(manual::SERVO_PINS[j], OUTPUT);
      digitalWrite(manual::SERVO_PINS[j], LOW);
    }
    pinMode(manual::ENABLE_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(manual::ENABLE_PIN), enableLostISR, FALLING);
    inputsReady = true;
  }
  if (manual::CAMERA_UART_CONFIRMED && manual::pinsDistinct()) {
    trialCamera.begin(manual::CAMERA_BAUD, SERIAL_8N1, manual::CAMERA_RX_PIN, manual::CAMERA_TX_PIN);
    trialUartReady = true;
  }
  Serial.printf("Assembled-arm manual calibration. outputs_config=%u pinout_confirmed=%u limits_confirmed=%u. HELP.\n",
    manual::OUTPUTS_ENABLED, manual::PINOUT_CONFIRMED, manual::LIMITS_CONFIRMED);
  status();
}
void loop() {
  controller.tick(millis(), enablePresent(), true);
  redTrial.tick(millis(), enablePresent());
  if (trialUartReady) for (uint16_t n = 0; n < 128 && trialCamera.available(); ++n) {
    Packet packet = {};
    if (trialParser.feed(char(trialCamera.read()), packet))
      redTrial.receive(packet, millis(), enablePresent());
  }
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
