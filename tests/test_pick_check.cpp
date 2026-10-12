// Actual sketch, fake GPIO/UART/clock only. No serial port or motor is opened.
#include <cassert>
#include <new>
#include <vector>
#include "../firmware/arm_calibration/arm_calibration.ino"

const JointPose park = {{1500,1200,1500,2100}};
bool same(const JointPose& a, const JointPose& b) { return !std::memcmp(&a, &b, sizeof(a)); }
bool state(const char* name) { return !std::strcmp(redTrial.stateName(), name); }
bool output(const char* text) { return Serial.log.find(text) != std::string::npos; }
void send(const char* text) {
  Serial.input = std::string(text) + "\n"; Serial.cursor = 0;
  while (Serial.available()) loop();
}
std::vector<Packet> queries() {
  LineParser parser; std::vector<Packet> result;
  for (char ch : trialCamera.log) { Packet p = {}; if (parser.feed(ch,p)) result.push_back(p); }
  assert(!parser.errors()); return result;
}
void fresh(uint32_t now = 0) {
  fakehw::clear(); fakehw::now = now; fakehw::levels[32] = HIGH;
  Serial = FakeSerial{}; trialCamera = HardwareSerial(1); io = EspJogIO{};
  inputsReady = trialUartReady = false; enableLossLatched = false;
  jogSettings = manual::settings();
  controller.~ManualJogController(); new (&controller) ManualJogController(jogSettings,io);
  redTrial.~RedPickTrial(); new (&redTrial) red_trial::RedPickTrial(controller,jogSettings,trialCamera);
  trialParser = LineParser{}; lineSize = 0; lineInvalid = false; setup(); loop();
  assert(!fakehw::attachCalls && !fakehw::writeCalls);
}
void arm() {
  send("SEED 1500,1200,1500,2100"); send("SUPPORTED"); send("ARM");
  assert(controller.outputsOn() && same(controller.commandedPose(),park));
}
Scene one(uint8_t cls = 3) {
  Scene s = {}; s.cameraBoot = 2546176631U; s.calibrationId = manual::CAMERA_CALIBRATION_ID;
  s.count = 1; s.objects[0] = {cls,1126,590,400}; return s;
}
Packet response(const Scene& s) {
  const Packet q = queries().back(); Packet p = {};
  p.type = 'D'; p.session = q.session; p.seq = q.seq;
  assert(encodeScene(s,p.payload,sizeof(p.payload))); return p;
}
void wire(const Packet& p) {
  char frame[kMaxFrame]; assert(encodePacket(p,frame,sizeof(frame)));
  trialCamera.input = frame; trialCamera.cursor = 0;
  while (trialCamera.available()) loop();
}
void step(bool openOnly = true) {
  const JointPose before = controller.commandedPose();
  fakehw::now += 20; loop();
  const JointPose after = controller.commandedPose();
  if (controller.outputsOn()) {
    if (openOnly) assert(after.us[3] == 2100); // CHECKPICK never closes.
    for (unsigned j = 0; j < 4; ++j) {
      assert(std::abs(int(after.us[j])-int(before.us[j])) <= 6);
      assert(after.us[j] >= jogSettings.minUs[j] && after.us[j] <= jogSettings.maxUs[j]);
    }
  }
}
void until(const char* wanted) {
  const uint32_t from = millis();
  while (!state(wanted)) { assert(redTrial.busy() && uint32_t(millis()-from) < 60000); step(); }
  assert(controller.state() == JogState::Holding && controller.outputsOn());
}
void waitPaused(uint32_t duration) {
  assert(state("PAUSED")); const unsigned writes = fakehw::writeCalls;
  const JointPose p = controller.commandedPose(); const uint32_t from = millis();
  while (uint32_t(millis()-from) < duration) step();
  assert(state("PAUSED") && redTrial.busy() && fakehw::writeCalls == writes);
  assert(same(p,controller.commandedPose()) && queries().size() == 1);
}
void begin(uint8_t cls = 3) {
  send("CHECKPICK"); assert(state("WAIT_CAMERA") && queries().size() == 1);
  wire(response(one(cls))); until("PAUSED");
  assert(output("CHECKPOINT name=SIDE_HIGH") && !std::strcmp(redTrial.actionName(),"CHECKPICK"));
}
void noRestart() {
  const unsigned writes = fakehw::writeCalls, attaches = fakehw::attachCalls;
  fakehw::levels[32] = HIGH; fakehw::failWrite = -1;
  for (unsigned i = 0; i < 30; ++i) step();
  send("NEXT"); send("BACK"); send("ARM");
  assert(!redTrial.busy() && !controller.outputsOn());
  assert(writes == fakehw::writeCalls && attaches == fakehw::attachCalls);
}
int main() {
  // The installed profile must really move at 300 us/s, not just print it.
  fresh(); arm(); send("HELP"); assert(output("rate 300 us/s"));
  const uint32_t moveStarted = millis();
  send("MOVE 1500,1500,1500,2100"); // Shoulder travels 300 us in 1 s (previously 3 s).
  for (uint32_t elapsed=20; elapsed<=1000; elapsed+=20) {
    fakehw::now = moveStarted + elapsed; loop();
    assert(controller.state()==(elapsed==1000 ? JogState::Holding : JogState::Jogging));
  }
  const JointPose hub = {{1500,1500,1500,2100}};
  assert(same(controller.commandedPose(),hub));
  const uint32_t jogStarted = millis();
  send("JOG 2 -90");
  for (uint32_t elapsed=20; elapsed<=300; elapsed+=20) {
    fakehw::now = jogStarted + elapsed; loop();
    assert(controller.state()==(elapsed==300 ? JogState::Holding : JogState::Jogging));
  }
  assert(controller.commandedPose().us[2]==1410);
  // A delayed loop still issues only one 20 ms step, never catches up in a jump.
  send("MOVE 1500,1500,1500,2100"); fakehw::now += 100; loop();
  assert(controller.commandedPose().us[2]==1416);
  send("ESTOP"); assert(!controller.outputsOn());

  // Verify the user-requested geometry independently of the sketch state machine.
  const auto map = camera_aim::sortCalibration();
  const auto settings = manual::settings();
  assert(manual::SIDE_APPROACH_MM == 25);
  for (uint8_t cls : {uint8_t(1),uint8_t(3)}) {
    const Detection d = {cls,1154,523,400}; uint8_t counts[2] = {}; counts[classBinIndex(cls)] = 1;
    camera_sort::Plan old = {}, side = {};
    assert(camera_sort::makePlan(d,counts,map,settings,old));
    assert(camera_sort::makeSidePlan(d,counts,map,settings,side));
    CalibratedPose h = {}, l = {};
    assert(interpolateGrid(map,1154,273,true,h) && interpolateGrid(map,1154,273,false,l));
    assert(same(side.poses[1],h.joints) && same(side.poses[2],l.joints));
    for (unsigned n=0;n<14;++n) if (n!=1 && n!=2) assert(same(side.poses[n],old.poses[n]));
    assert(side.poses[2].us[3]==2100 && side.poses[3].us[3]==2100 && side.poses[4].us[3]==1250);
    int32_t y;
    assert(camera_sort::sideStart({cls,750,500,400},map,y) && y==250);
    assert(!camera_sort::sideStart({cls,750,499,400},map,y));
    assert(camera_sort::sideStart({cls,750,800,400},map,y) && y==550);
    assert(!camera_sort::sideStart({cls,750,INT32_MIN,400},map,y));
    assert(!camera_sort::sideStart({cls,INT32_MAX,500,400},map,y));
    auto stricter = settings; stricter.minUs[2]=550;
    assert(camera_sort::makePlan({cls,200,500,400},counts,map,stricter,old));
    assert(!camera_sort::makeSidePlan({cls,200,500,400},counts,map,stricter,side));
  }
  // A neighbour can satisfy centre spacing yet obstruct the low entry segment.
  Scene blocked = one(); blocked.count=2;
  blocked.objects[0]={3,750,800,400}; blocked.objects[1]={1,750,500,400};
  uint8_t blockedCounts[2]={};
  assert(!camera_sort::inspectSideScene(blocked,map,blockedCounts));
  assert(!camera_sort::sidePathClear(blocked.objects[0],blocked,map));
  Scene clear=blocked; clear.objects[1]={1,1080,600,400};
  assert(!camera_sort::inspectSideScene(clear,map,blockedCounts));
  assert(camera_sort::sidePathClear(clear.objects[0],clear,map));

  fresh(); send("CHECKPICK"); send("NEXT"); send("BACK");
  assert(!redTrial.busy() && queries().empty() && !fakehw::writeCalls);
  arm(); trialUartReady = false; send("CHECKPICK");
  assert(!redTrial.busy() && queries().empty() && fakehw::writeCalls == 4);
  fresh(); arm(); send("MOVE 1501,1200,1500,2100"); while (controller.state()!=JogState::Holding) step();
  send("CHECKPICK"); assert(output("START_PARK_REQUIRED") && queries().empty());

  for (unsigned bad = 0; bad < 7; ++bad) {
    fresh(); arm(); send("CHECKPICK"); Scene s = one();
    if (bad == 0) s.count = 0;
    if (bad == 1) { s.count = 2; s.objects[1] = {1,410,693,400}; }
    if (bad == 2) s.objects[0].x10 = 1301;
    if (bad == 4) ++s.calibrationId;
    Packet p = response(s);
    if (bad == 3) std::strcpy(p.payload,"2546176631,1573755510,1;3,1126,340,0");
    if (bad == 5) std::strcpy(p.payload,"0,1573755510,1;3,1126,340,400");
    if (bad == 6) std::strcpy(p.payload,"2546176631,1573755510,1;2,1126,340,400");
    // Malformed payloads are normally rejected by the parser; also exercise
    // the coordinator's independent validation without passing that parser.
    if (bad == 3 || bad == 5 || bad == 6) redTrial.receive(p,millis(),true);
    else wire(p);
    assert(state("FAILED") && fakehw::writeCalls == 4 && same(park,controller.commandedPose()));
  }

  for (uint8_t cls : {uint8_t(1),uint8_t(3)}) for (unsigned stopAt = 1; stopAt <= 3; ++stopAt) {
    fresh(); arm(); begin(cls);
    camera_sort::Plan expected = {}; uint8_t counts[2] = {}; counts[classBinIndex(cls)] = 1;
    assert(camera_sort::makeSidePlan(one(cls).objects[0],counts,camera_aim::sortCalibration(),jogSettings,expected));
    for (unsigned point = 1; point <= stopAt; ++point) {
      assert(same(controller.commandedPose(),expected.poses[point]));
      waitPaused(65000);
      for (const char* cmd : {"JOG 1 -100","MOVE 1500,1500,1500,2100","SORT","AIMRED","TRIALRED","CHECKPICK"}) {
        send(cmd); assert(state("PAUSED") && same(controller.commandedPose(),expected.poses[point]));
      }
      send("POSE"); send("HOLD"); send("STATUS");
      if (point < stopAt) {
        send("NEXT"); send("NEXT"); send("BACK"); // Repeated commands are refused, never queued.
        assert(output("command not queued")); until("PAUSED");
      }
    }
    if (stopAt == 3) { send("NEXT"); assert(state("PAUSED") && output("LOW_REACHED")); }
    send("BACK"); std::vector<JointPose> reached;
    const uint32_t start = millis();
    while (!state("DONE")) {
      assert(uint32_t(millis()-start) < 60000); step();
      if (state("DWELL") && (reached.empty() || !same(reached.back(),controller.commandedPose())))
        reached.push_back(controller.commandedPose());
    }
    assert(reached.size() == stopAt + 1);
    for (unsigned n = 0; n < stopAt; ++n) assert(same(reached[n],expected.poses[stopAt-1-n]));
    assert(same(reached.back(),park) && same(controller.commandedPose(),park));
    assert(queries().size() == 1 && !redTrial.busy() && controller.outputsOn());
    assert(!redTrial.sortedRed() && !redTrial.sortedBlue() && !redTrial.sortAttempts());
    assert(output("CHECK_RESULT returned_park_command=1 gripper_open=1"));
    assert(!output("name=GRIP") && !output("name=BIN_") && !output("SORT_PICK_DONE"));
  }
  // Cancellation must work both in a held checkpoint and during each descent.
  for (unsigned position = 0; position < 5; ++position) for (unsigned cause = 0; cause < 4; ++cause) {
    fresh(); arm(); begin();
    if (position >= 1) { send("NEXT"); if (position >= 2) until("PAUSED"); }
    if (position >= 3) { send("NEXT"); if (position >= 4) until("PAUSED"); }
    const Packet late = response(one());
    if (cause == 0) send("DISARM");
    if (cause == 1) send("ESTOP");
    if (cause == 2) send("RESET");
    if (cause == 3) { fakehw::levels[32] = LOW; enableLostISR(); step(); }
    wire(late); noRestart();
  }
  fresh(); arm(); begin(); send("NEXT"); fakehw::failWrite = 25; step(); noRestart();
  fresh(); arm(); begin();
  while (controller.outputsOn()) step();
  assert(controller.error() == JogError::LeaseExpired && !redTrial.busy()); noRestart();
  for (uint32_t start : {0U,UINT32_MAX-4000U}) {
    fresh(start); arm(); const uint32_t started = millis(); begin();
    while (uint32_t(millis()-started) < 899000) {
      if (controller.holdRemainingMs(millis()) < 200000) send("HOLD");
      step();
    }
    while (redTrial.busy()) step();
    assert(!controller.outputsOn() && !std::strcmp(redTrial.error(),"CHECK_TIME_LIMIT")); noRestart();
  }
  fresh(); arm(); send("CHECKPICK");
  Packet stale = response(one()); ++stale.session; wire(stale);
  assert(state("WAIT_CAMERA") && fakehw::writeCalls == 4);
  while (redTrial.busy()) step();
  assert(!std::strcmp(redTrial.error(),"CAMERA_TIMEOUT") && same(controller.commandedPose(),park));

  for (const Scene& s : {blocked, []{ auto s=one(); s.objects[0].y10=499; return s; }()}) {
    fresh(); arm(); send("SORT"); wire(response(s));
    assert(state("FAILED") && fakehw::writeCalls==4 && same(controller.commandedPose(),park));
    assert(output(s.count==2?"SIDE_PATH_BLOCKED":"SIDE_APPROACH_OUTSIDE_AREA"));
  }
  // Actual command dispatch: both colour orders, side entry before closing,
  // lift at the target, fresh PARK frame, unchanged bin choice and counters.
  for (bool blueFirst : {false,true}) {
    fresh(); arm(); send("SORT"); Scene s=one(); s.count=2;
    s.objects[0]={uint8_t(blueFirst?3:1),400,650,400};
    s.objects[1]={uint8_t(blueFirst?1:3),1000,650,400};
    for (unsigned pick=0;pick<2;++pick) {
      wire(response(s)); assert(state("MOVING"));
      uint8_t counts[2]={}; assert(!camera_sort::inspectSideScene(s,map,counts));
      camera_sort::Plan expected = {};
      assert(camera_sort::makeSidePlan(s.objects[0],counts,map,jogSettings,expected));
      unsigned endpoint=0; const uint32_t started=millis();
      while (!state("VERIFY_CAMERA")) {
        assert(redTrial.busy() && uint32_t(millis()-started)<100000); step(false);
        if (endpoint<14 && same(controller.commandedPose(),expected.poses[endpoint])) ++endpoint;
      }
      assert(endpoint==14 && same(controller.commandedPose(),park));
      if (s.count==2) s.objects[0]=s.objects[1]; --s.count;
    }
    wire(response(s)); assert(state("DONE") && controller.outputsOn());
    assert(redTrial.sortedRed()==1 && redTrial.sortedBlue()==1 && redTrial.sortAttempts()==2);
    assert(queries().size()==3 && output("direction=+Y") && output("distance_mm=25"));
    assert(output("name=SIDE_HIGH") && output("name=SIDE_LOW") && output("name=ENTER_Y"));
  }

  // Show the side-route predictions for the synthetic blue target, not measurements.
  camera_sort::Plan p = {}; uint8_t counts[2] = {0,1};
  assert(camera_sort::makeSidePlan(one().objects[0],counts,camera_aim::sortCalibration(),jogSettings,p));
  for (unsigned i = 1; i <= 3; ++i)
    std::printf("PREDICTED %s %u,%u,%u,%u\n",i==1?"SIDE_HIGH":i==2?"SIDE_LOW":"TARGET_LOW",
      p.poses[i].us[0],p.poses[i].us[1],p.poses[i].us[2],p.poses[i].us[3]);
  std::puts("CHECKPICK actual sketch with fake hardware: PASS; no physical accuracy claim");
}
