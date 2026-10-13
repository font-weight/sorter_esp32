#include <Arduino.h>
#include <Protocol.h>
#include "Config.h"
#include "ArmMap.h"
#include "PoseStore.h"
#include "StartButton.h"
#include "SortController.h"

using namespace sorter;
volatile bool enableLossLatched = false;
bool inputsReady = false;
void ARDUINO_ISR_ATTR enableLostISR() { enableLossLatched = true; }
bool enablePresent() { return inputsReady && !enableLossLatched && digitalRead(app::ENABLE_PIN)==HIGH; }

class ServoIO : public JogIO {
 public:
  bool attached[4] = {};
  bool enableAt(const JointPose& pose) override {
    for (uint8_t j=0;j<4;++j) {
      if (!enablePresent()) { detachOutputs(); return false; }
      attached[j] = ledcAttach(app::SERVO_PINS[j],50,14);
      if (!attached[j] || !enablePresent() || !write(j,pose.us[j])) { detachOutputs(); return false; }
    }
    return true;
  }
  bool writePose(const JointPose& pose) override {
    for (uint8_t j=0;j<4;++j)
      if (!attached[j] || !enablePresent() || !write(j,pose.us[j])) return false;
    return true;
  }
  void detachOutputs() override {
    for (uint8_t j=0;j<4;++j) if (attached[j]) {
      ledcWrite(app::SERVO_PINS[j],0); ledcDetach(app::SERVO_PINS[j]);
      pinMode(app::SERVO_PINS[j],OUTPUT); digitalWrite(app::SERVO_PINS[j],LOW); attached[j]=false;
    }
  }
 private:
  static bool write(uint8_t j,uint16_t us) {
    return ledcWrite(app::SERVO_PINS[j],(uint32_t(us)*16384U+10000U)/20000U);
  }
};

JogSettings motionSettings = app::settings();
ServoIO servoIO;
ManualJogController motion(motionSettings,servoIO);
HardwareSerial camera(1);
app::SortController sorterController(motion,motionSettings,camera);
app::PoseStore poseStore;
app::StartButton startButton;
LineParser cameraParser;
enum class RunState { WaitEnable, Ready, Hub, Park, Settling, Sorting, Stopped, Fault };
RunState runState = RunState::WaitEnable;
const char* appError = "NONE";
JointPose lastCommand = arm_map::PARK;
bool storageReady = false, uartReady = false;
uint32_t savedAt = 0, settledAt = 0, renewedAt = 0;
char inputLine[32];
uint8_t inputSize = 0;
bool badLine = false;

const char* runName() {
  switch (runState) {
    case RunState::WaitEnable: return "WAIT_ENABLE";
    case RunState::Ready: return "READY";
    case RunState::Hub: return "RETURN_HUB";
    case RunState::Park: return "RETURN_PARK";
    case RunState::Settling: return "PARK_SETTLING";
    case RunState::Sorting: return "SORTING";
    case RunState::Stopped: return "STOPPED";
    default: return "FAULT";
  }
}
bool busy() { return runState==RunState::Hub || runState==RunState::Park ||
                    runState==RunState::Settling || runState==RunState::Sorting; }
void status() {
  Serial.printf("STATUS state=%s error=%s outputs=%u permit=%u storage=%u rate_us_s=%u pose_us=%u,%u,%u,%u sort=%s sort_error=%s red=%u blue=%u\n",
    runName(),appError,motion.outputsOn(),enablePresent(),storageReady,app::RATE_US_PER_SECOND,
    lastCommand.us[0],lastCommand.us[1],lastCommand.us[2],lastCommand.us[3],
    sorterController.stateName(),sorterController.error(),sorterController.sortedRed(),sorterController.sortedBlue());
}
void stop(const char* reason,bool fault) {
  if (motion.outputsOn()) lastCommand=motion.commandedPose();
  sorterController.cancel(reason); motion.disarm();
  runState=fault ? RunState::Fault : RunState::Stopped; appError=reason;
  // PWM is already off before a blocking flash commit is attempted.
  if (storageReady && !poseStore.save(lastCommand)) { storageReady=false; runState=RunState::Fault; appError="POSE_SAVE_FAILED"; }
  status();
}
bool restore() {
  if (!storageReady || !inputsReady || digitalRead(app::ENABLE_PIN)!=HIGH) return false;
  enableLossLatched=false; // Only initial restore or an explicit new SORT reaches here.
  motion.disarm(); motion.reset();
  if (!motion.seed(lastCommand) || !motion.confirmSupported() || !motion.arm(millis(),enablePresent(),true)) {
    stop("RESTORE_FAILED",true); return false;
  }
  appError="NONE"; runState=RunState::Ready; renewedAt=millis();
  Serial.printf("RESTORED command_us=%u,%u,%u,%u; saved commands, no position feedback\n",
    lastCommand.us[0],lastCommand.us[1],lastCommand.us[2],lastCommand.us[3]);
  return true;
}
void requestSort() {
  if (busy()) { Serial.println("SORT_REFUSED reason=BUSY; request not queued"); return; }
  if (!storageReady) { Serial.println("SORT_REFUSED reason=POSE_STORAGE_UNAVAILABLE"); return; }
  if (!inputsReady || digitalRead(app::ENABLE_PIN)!=HIGH) { Serial.println("SORT_REFUSED reason=ENABLE_OFF"); return; }
  if (!motion.outputsOn() || !enablePresent()) {
    if (!restore()) return;
  }
  appError="NONE";
  if (app::PoseStore::same(motion.commandedPose(),arm_map::PARK)) {
    runState=RunState::Settling; settledAt=millis();
  } else {
    // Preserve the gripper command while raising toward HUB; PARK then opens it.
    JointPose hub=arm_map::HUB; hub.us[3]=motion.commandedPose().us[3];
    if (!motion.movePose(hub,millis(),enablePresent(),true)) { stop("RETURN_HUB_FAILED",true); return; }
    runState=RunState::Hub;
  }
  Serial.println("SORT_ACCEPTED");
}
void command(const char* line) {
  if (!strcmp(line,"SORT")) requestSort();
  else if (!strcmp(line,"STOP") || !strcmp(line,"ESTOP")) stop("OPERATOR_STOP",false);
  else if (!strcmp(line,"STATUS")) status();
  else if (!strcmp(line,"HELP")) {
    Serial.println("SORT | STOP | ESTOP | STATUS | HELP");
    Serial.println("SORT or a new button press starts one batch. GPIO18 button to GND, internal pull-up.");
    Serial.println("ENABLE32 must be high. After STOP/ENABLE loss only a new SORT/button restarts.");
    Serial.println("Startup restores saved commands (first boot PARK); no interrupted batch resumes.");
    Serial.printf("Rate %u us/s; side entry %ld mm; pose checkpoint %lu ms + reached poses.\n",
      app::RATE_US_PER_SECOND,(long)app::SIDE_APPROACH_MM,(unsigned long)app::POSE_SAVE_INTERVAL_MS);
  } else Serial.println("UNKNOWN_COMMAND; HELP");
}
void setup() {
  Serial.begin(app::USB_BAUD);
  if (!app::pinsDistinct()) { runState=RunState::Fault; appError="PIN_CONFLICT"; status(); return; }
  for (uint8_t j=0;j<4;++j) { pinMode(app::SERVO_PINS[j],OUTPUT); digitalWrite(app::SERVO_PINS[j],LOW); }
  pinMode(app::ENABLE_PIN,INPUT_PULLDOWN);
  pinMode(app::SORT_BUTTON_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(app::ENABLE_PIN),enableLostISR,FALLING);
  inputsReady=true;
  startButton.begin(digitalRead(app::SORT_BUTTON_PIN)==LOW,millis());
  camera.begin(app::CAMERA_BAUD,SERIAL_8N1,app::CAMERA_RX_PIN,app::CAMERA_TX_PIN); uartReady=true;
  bool empty=false;
  storageReady=poseStore.begin() && poseStore.load(lastCommand,empty);
  if (storageReady && empty) storageReady=poseStore.save(lastCommand);
  if (!storageReady) { runState=RunState::Fault; appError="POSE_STORAGE_INVALID"; status(); return; }
  savedAt=millis();
  Serial.println(empty ? "BOOT first_start=PARK" : "BOOT saved_pose_loaded");
  if (digitalRead(app::ENABLE_PIN)==HIGH) restore();
  status();
}
void loop() {
  if (runState==RunState::WaitEnable && storageReady && digitalRead(app::ENABLE_PIN)==HIGH) restore();
  const bool wasOn=motion.outputsOn();
  motion.tick(millis(),enablePresent(),true);
  if (wasOn) lastCommand=motion.commandedPose();
  if (wasOn && !motion.outputsOn()) stop(enablePresent() ? "MOTION_STOPPED" : "ENABLE_LOST",true);

  if (motion.outputsOn() && storageReady &&
      (motion.state()==JogState::Holding || uint32_t(millis()-savedAt)>=app::POSE_SAVE_INTERVAL_MS)) {
    if (!poseStore.save(lastCommand)) { storageReady=false; stop("POSE_SAVE_FAILED",true); }
    else savedAt=millis();
  }
  // Continue holding while ready, without operator KEEPALIVE commands.
  if (motion.outputsOn() && motion.state()==JogState::Holding && uint32_t(millis()-renewedAt)>=1000) {
    motion.hold(millis(),enablePresent(),true); renewedAt=millis();
  }
  if (runState==RunState::Hub && motion.state()==JogState::Holding) {
    if (motion.movePose(arm_map::PARK,millis(),enablePresent(),true)) runState=RunState::Park;
    else stop("RETURN_PARK_FAILED",true);
  } else if (runState==RunState::Park && motion.state()==JogState::Holding) {
    runState=RunState::Settling; settledAt=millis();
  } else if (runState==RunState::Settling && uint32_t(millis()-settledAt)>=app::PARK_SETTLE_MS) {
    if (sorterController.start(millis(),uartReady,enablePresent())) runState=RunState::Sorting;
    else { runState=RunState::Fault; appError="SORT_START_FAILED"; }
  }
  sorterController.tick(millis(),enablePresent());
  for (unsigned n=0;uartReady && n<128 && camera.available();++n) {
    Packet p={}; if (cameraParser.feed(char(camera.read()),p)) sorterController.receive(p,millis(),enablePresent());
  }
  if (runState==RunState::Sorting && !sorterController.busy()) {
    if (!strcmp(sorterController.stateName(),"DONE")) { runState=RunState::Ready; appError="NONE"; }
    else if (!motion.outputsOn()) { stop(sorterController.error(),true); }
    else { runState=RunState::Fault; appError=sorterController.error(); }
    status();
  }
  if (inputsReady && startButton.tick(digitalRead(app::SORT_BUTTON_PIN)==LOW,millis())) requestSort();
  for (unsigned n=0;n<64 && Serial.available();++n) {
    const char ch=char(Serial.read());
    if (ch=='\r') continue;
    if (ch=='\n') {
      if (!badLine) { inputLine[inputSize]=0; command(inputLine); }
      else Serial.println("INVALID_COMMAND");
      inputSize=0; badLine=false;
    } else if (!badLine) {
      if (ch<32 || ch>126 || inputSize+1>=sizeof(inputLine)) badLine=true;
      else inputLine[inputSize++]=ch;
    }
  }
  delay(1);
}
