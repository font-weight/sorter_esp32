#include <Arduino.h>
#include <Protocol.h>
#include "LocalConfig.h"

// Unloaded single-servo bench only. There is NO voltage or position feedback.
// CONFIRM is an operator check, not an electrical POWER_GOOD measurement.
// DISARM before disconnecting/restoring motor power or changing any wiring.
bool attached = false, faultLatched = false, supplyConfirmed = false;
uint16_t currentUs = bench::INITIAL_US, targetUs = bench::INITIAL_US;
uint32_t confirmedAt = 0, startedAt = 0, lastAction = 0, lastWrite = 0, lastLoop = 0;
char commandLine[80];
uint8_t commandLength = 0;
bool commandOverflow = false;

void stopOutput(const char* reason, bool fault) {
  if (attached) {
    ledcWrite(bench::SERVO_PIN, 0);
    ledcDetach(bench::SERVO_PIN);
    pinMode(bench::SERVO_PIN, OUTPUT);
    digitalWrite(bench::SERVO_PIN, LOW);
  }
  attached = false; supplyConfirmed = false; targetUs = currentUs;
  if (fault) faultLatched = true;
  Serial.print("DISARMED "); Serial.println(reason);
}
bool writePulse(uint16_t us) {
  return attached && ledcWrite(bench::SERVO_PIN,
      (uint32_t(us) * 16384U + 10000U) / 20000U);
}
void status() {
  const bool confirmationValid = supplyConfirmed &&
      uint32_t(millis() - confirmedAt) < bench::CONFIRM_VALID_MS;
  const uint32_t idleElapsed = uint32_t(millis() - lastAction);
  const uint32_t sessionElapsed = uint32_t(millis() - startedAt);
  const uint32_t idleLeft = attached && idleElapsed < bench::INACTIVITY_MS ? bench::INACTIVITY_MS - idleElapsed : 0;
  const uint32_t sessionLeft = attached && sessionElapsed < bench::MAX_SESSION_MS ? bench::MAX_SESSION_MS - sessionElapsed : 0;
  Serial.printf("STATUS mode=ONE_UNLOADED_SERVO gpio=%u outputs_config=%u armed=%u fault=%u "
                "confirmed=%u us=%u target=%u supply_monitor=NONE idle_left_s=%lu session_left_s=%lu\n",
                bench::SERVO_PIN, bench::OUTPUTS_ENABLED, attached, faultLatched,
                confirmationValid, currentUs, targetUs,
                (unsigned long)((idleLeft + 999U) / 1000U), (unsigned long)((sessionLeft + 999U) / 1000U));
}
void handleCommand(const char* line) {
  const uint32_t now = millis();
  if (attached && (!strcmp(line, "STATUS") || !strcmp(line, "HELP"))) lastAction = now;
  if (!strcmp(line, "HELP")) {
    Serial.println("ONE unloaded servo, GPIO25. External power with common GND. No power monitoring.");
    Serial.printf("STATUS | CONFIRM | ARM 1500 | US <%u..%u, step <=%u us> | DISARM | ESTOP | RESET\n",
      bench::MIN_US, bench::MAX_US, bench::MAX_STEP_US);
    Serial.println("CONFIRM: measured supply, clear shaft, power disconnect available. Initial ARM may jump.");
    Serial.printf("CONFIRM valid %lu s; idle %lu s; session %lu s from ARM. STATUS/HELP renew idle only.\n",
      (unsigned long)(bench::CONFIRM_VALID_MS / 1000U), (unsigned long)(bench::INACTIVITY_MS / 1000U),
      (unsigned long)(bench::MAX_SESSION_MS / 1000U));
    Serial.println("DISARM before every power/wiring change.");
  } else if (!strcmp(line, "STATUS")) status();
  else if (!strcmp(line, "DISARM")) stopOutput("operator", false);
  else if (!strcmp(line, "ESTOP")) stopOutput("ESTOP; RESET required", true);
  else if (!strcmp(line, "RESET")) {
    if (!attached) { faultLatched = false; supplyConfirmed = false; Serial.println("Reset; CONFIRM and ARM required"); }
  } else if (!strcmp(line, "CONFIRM")) {
    if (attached || faultLatched) { Serial.println("REFUSED: disarm/reset first"); return; }
    supplyConfirmed = true; confirmedAt = now;
    Serial.printf("Operator confirmation valid for %lu s. No motor output yet.\n", (unsigned long)(bench::CONFIRM_VALID_MS / 1000U));
  } else if (!strncmp(line, "ARM ", 4)) {
    const bool confirmed = supplyConfirmed && uint32_t(now - confirmedAt) < bench::CONFIRM_VALID_MS;
    supplyConfirmed = false; // Single-use, including refused attempts.
    if (!bench::OUTPUTS_ENABLED || attached || faultLatched || !confirmed) {
      Serial.println("REFUSED: outputs disabled, already armed, fault, or fresh CONFIRM missing"); return;
    }
    uint32_t us = 0;
    if (!sorter::parseU32(line + 4, us) || us != bench::INITIAL_US) {
      Serial.println("REFUSED: initial command must be ARM 1500"); return;
    }
    currentUs = targetUs = uint16_t(us);
    attached = ledcAttach(bench::SERVO_PIN, 50, 14);
    if (!attached || !writePulse(currentUs)) { stopOutput("output failure", true); return; }
    startedAt = lastAction = lastWrite = lastLoop = now;
    status();
  } else if (!strncmp(line, "US ", 3)) {
    uint32_t us = 0;
    if (!attached || !sorter::parseU32(line + 3, us) || us < bench::MIN_US || us > bench::MAX_US) {
      Serial.println("REFUSED: not armed or pulse outside bench range"); return;
    }
    const int32_t delta = int32_t(us) - currentUs;
    if (delta < -int32_t(bench::MAX_STEP_US) || delta > int32_t(bench::MAX_STEP_US)) {
      Serial.printf("REFUSED: step must be <=%u us from current command\n", bench::MAX_STEP_US); return;
    }
    targetUs = uint16_t(us); lastAction = now; status();
  } else Serial.println("Unknown command; HELP");
}
void setup() {
  Serial.begin(115200);
  // No PWM or servo GPIO configuration at startup.
  lastLoop = millis();
  Serial.printf("Single unloaded SG90 bench. outputs_config=%u. No automatic movement. HELP.\n", bench::OUTPUTS_ENABLED);
  status();
}
void loop() {
  const uint32_t now = millis();
  if (attached) {
    if (uint32_t(now - lastLoop) > bench::MAX_LOOP_GAP_MS) stopOutput("late control loop", true);
    else if (uint32_t(now - startedAt) >= bench::MAX_SESSION_MS) stopOutput("session time limit", false);
    else if (uint32_t(now - lastAction) >= bench::INACTIVITY_MS) stopOutput("command inactivity", false);
    else if (uint32_t(now - lastWrite) >= 20) {
      const uint16_t step = uint16_t(bench::RATE_US_PER_SECOND * 20U / 1000U);
      uint16_t next = currentUs;
      if (targetUs > currentUs) next += uint16_t(targetUs - currentUs) < step ? uint16_t(targetUs - currentUs) : step;
      else if (targetUs < currentUs) next -= uint16_t(currentUs - targetUs) < step ? uint16_t(currentUs - targetUs) : step;
      if (!writePulse(next)) stopOutput("output failure", true);
      else currentUs = next;
      lastWrite = now;
    }
  }
  lastLoop = now;
  for (uint8_t n = 0; n < 64 && Serial.available(); ++n) {
    const char ch = char(Serial.read());
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (!commandOverflow) { commandLine[commandLength] = 0; handleCommand(commandLine); }
      else Serial.println("Oversized command discarded");
      commandLength = 0; commandOverflow = false;
    } else if (!commandOverflow) {
      if (commandLength + 1 < sizeof(commandLine)) commandLine[commandLength++] = ch;
      else commandOverflow = true;
    }
  }
  delay(1);
}
