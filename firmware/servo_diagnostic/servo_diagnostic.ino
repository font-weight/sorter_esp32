#include <Arduino.h>
#include <Protocol.h>
#include "LocalConfig.h"

bool inputsReady = false, attached = false, faultLatched = false;
volatile bool inputLossLatched = false;
void ARDUINO_ISR_ATTR inputLostISR() { inputLossLatched = true; }
uint8_t activeIndex = 0;
uint16_t currentUs = 1500, targetUs = 1500;
uint32_t lastAction = 0, lastWrite = 0, lastLoop = 0;
char commandLine[80];
uint8_t commandLength = 0;
bool commandOverflow = false;

bool permitNow() {
  return inputsReady && !inputLossLatched && digitalRead(diagnostic::ENABLE_PIN) == HIGH &&
    digitalRead(diagnostic::POWER_GOOD_PIN) == HIGH;
}
bool writePulse(uint16_t us) {
  return attached && permitNow() && ledcWrite(diagnostic::SERVO_PINS[activeIndex],
      (uint32_t(us)*16384U + 10000U)/20000U);
}
void stopOutput(const char* reason, bool fault) {
  if (attached) {
    ledcWrite(diagnostic::SERVO_PINS[activeIndex], 0);
    ledcDetach(diagnostic::SERVO_PINS[activeIndex]);
    pinMode(diagnostic::SERVO_PINS[activeIndex], OUTPUT);
    digitalWrite(diagnostic::SERVO_PINS[activeIndex], LOW);
  }
  attached = false; targetUs = currentUs;
  if (fault) faultLatched = true;
  Serial.print("DISARMED "); Serial.println(reason);
}
void status() {
  Serial.printf("STATUS outputs_config=%u pinout_confirmed=%u armed=%u fault=%u index=%u us=%u target=%u permit=%u\n",
    diagnostic::OUTPUTS_ENABLED, diagnostic::PINOUT_CONFIRMED, attached, faultLatched,
    activeIndex, currentUs, targetUs, permitNow());
}
bool parseArm(const char* line, uint32_t& index, uint32_t& us) {
  // Strict two unsigned decimal fields; never let '-1' wrap into an unsigned value.
  char temp[32];
  if (strlen(line) >= sizeof(temp)) return false;
  strcpy(temp, line);
  char* space = strchr(temp, ' ');
  if (!space) return false;
  *space++ = 0;
  return sorter::parseU32(temp, index) && sorter::parseU32(space, us);
}
void handleCommand(const char* line) {
  if (!strcmp(line, "HELP")) {
    Serial.println("Bench only; disconnect linkage. ARM <index0..3> <initial_us>; US <target_us>; DISARM; ESTOP; RESET; STATUS.");
    Serial.println("No sweep. Initial positioning can jump. Only one servo output attaches. US steps <=25 us.");
  } else if (!strcmp(line, "STATUS")) status();
  else if (!strcmp(line, "DISARM")) stopOutput("operator", false);
  else if (!strcmp(line, "ESTOP")) stopOutput("ESTOP; RESET required", true);
  else if (!strcmp(line, "RESET")) {
    if (!attached) { faultLatched = false; Serial.println("Reset; a new ARM is still required"); }
  } else if (!strncmp(line, "ARM ", 4)) {
    uint32_t index = 0, us = 0;
    if (!diagnostic::OUTPUTS_ENABLED || !diagnostic::PINOUT_CONFIRMED || !diagnostic::configurationValid()) {
      Serial.println("REFUSED: hardware configuration disabled/unconfirmed"); return;
    }
    if (!attached && !faultLatched && inputsReady && digitalRead(diagnostic::ENABLE_PIN) == HIGH &&
        digitalRead(diagnostic::POWER_GOOD_PIN) == HIGH) inputLossLatched = false;
    if (attached || faultLatched || !permitNow()) {
      Serial.println("REFUSED: already armed, latched fault, or physical enable/power absent"); return;
    }
    if (!parseArm(line + 4, index, us) || index > 3 ||
        us < diagnostic::MIN_US[index] || us > diagnostic::MAX_US[index]) {
      Serial.println("REFUSED: ARM index or initial pulse outside configured bench range"); return;
    }
    activeIndex = uint8_t(index); currentUs = targetUs = uint16_t(us);
    attached = ledcAttach(diagnostic::SERVO_PINS[activeIndex], 50, 14);
    if (!attached || !writePulse(currentUs)) { stopOutput("output failure", true); return; }
    lastAction = lastWrite = lastLoop = millis(); status();
  } else if (!strncmp(line, "US ", 3)) {
    uint32_t us = 0;
    if (!attached || !permitNow() || !sorter::parseU32(line + 3, us) ||
        us < diagnostic::MIN_US[activeIndex] || us > diagnostic::MAX_US[activeIndex]) {
      Serial.println("REFUSED: not armed, no physical permit, or pulse out of range"); return;
    }
    const int32_t delta = int32_t(us) - currentUs;
    if (delta < -int32_t(diagnostic::MAX_COMMAND_STEP_US) || delta > int32_t(diagnostic::MAX_COMMAND_STEP_US)) {
      Serial.println("REFUSED: change must be <=25 us from current command"); return;
    }
    targetUs = uint16_t(us); lastAction = millis(); status();
  } else Serial.println("Unknown command; HELP");
}
void setup() {
  Serial.begin(115200);
  if (diagnostic::PINOUT_CONFIRMED && diagnostic::configurationValid()) {
    pinMode(diagnostic::ENABLE_PIN, INPUT_PULLDOWN);
    pinMode(diagnostic::POWER_GOOD_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(diagnostic::ENABLE_PIN), inputLostISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(diagnostic::POWER_GOOD_PIN), inputLostISR, FALLING);
    inputsReady = true;
  }
  lastLoop = millis();
  Serial.println("SG90 bench diagnostic. Outputs disabled by default. No automatic movement. HELP.");
  status();
}
void loop() {
  const uint32_t now = millis();
  if (attached) {
    if (!permitNow()) stopOutput("enable or motor supply lost; RESET then ARM", true);
    else if (uint32_t(now-lastLoop) > diagnostic::MAX_LOOP_GAP_MS) stopOutput("late control loop", true);
    else if (uint32_t(now-lastAction) >= diagnostic::INACTIVITY_MS) stopOutput("10 s command inactivity", false);
    else if (uint32_t(now-lastWrite) >= 20) {
      const uint16_t step = uint16_t(diagnostic::RATE_US_PER_SECOND * 20U / 1000U);
      uint16_t next = currentUs;
      if (targetUs > currentUs) next += min(uint16_t(targetUs-currentUs), step);
      else if (targetUs < currentUs) next -= min(uint16_t(currentUs-targetUs), step);
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
