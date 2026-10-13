// Compile and run the actual sketch with in-memory GPIO, UART, clock and NVS.
// No physical hardware, serial port or user data is accessed.
#include <cassert>
#include <new>
#include <vector>
#include "../firmware/sorter_node/sorter_node.ino"

static bool same(const JointPose& a,const JointPose& b) { return app::PoseStore::same(a,b); }
static bool output(const char* text) { return Serial.log.find(text)!=std::string::npos; }
static bool sortState(const char* text) { return !std::strcmp(sorterController.stateName(),text); }
static void reboot(bool erase=true,bool enable=true,bool pressed=false,uint32_t now=0) {
  if (erase) fakenvs::clear();
  fakehw::clear(); fakehw::now=now;
  fakehw::levels[app::ENABLE_PIN]=enable?HIGH:LOW;
  fakehw::levels[app::SORT_BUTTON_PIN]=pressed?LOW:HIGH;
  Serial=FakeSerial{}; camera=HardwareSerial(1); servoIO=ServoIO{};
  motionSettings=app::settings();
  motion.~ManualJogController(); new (&motion) ManualJogController(motionSettings,servoIO);
  sorterController.~SortController(); new (&sorterController) app::SortController(motion,motionSettings,camera);
  poseStore.~PoseStore(); new (&poseStore) app::PoseStore;
  startButton=app::StartButton{}; cameraParser=LineParser{};
  inputsReady=enableLossLatched=storageReady=uartReady=false;
  runState=RunState::WaitEnable; appError="NONE"; lastCommand=arm_map::PARK;
  savedAt=settledAt=renewedAt=0; inputSize=0; badLine=false;
  setup();
}
static void advance(uint32_t duration) {
  const uint32_t start=millis();
  while (uint32_t(millis()-start)<duration) {
    const uint32_t remaining=duration-uint32_t(millis()-start);
    fakehw::now+=remaining>20?19:remaining-1;
    loop();
  }
}
static void send(const char* text) {
  Serial.input=std::string(text)+"\n"; Serial.cursor=0;
  while (Serial.available()) loop();
}
static std::vector<Packet> queries() {
  LineParser parser; std::vector<Packet> result;
  for (char c : camera.log) { Packet p; if (parser.feed(c,p)) result.push_back(p); }
  assert(!parser.errors()); return result;
}
static Scene parts(bool blueFirst=false) {
  Scene scene; scene.cameraBoot=2546176631U; scene.calibrationId=app::CAMERA_CALIBRATION_ID;
  scene.count=2;
  scene.objects[0]={uint8_t(blueFirst?3:1),400,650,400};
  scene.objects[1]={uint8_t(blueFirst?1:3),1000,650,400}; return scene;
}
static Packet response(const Scene& scene) {
  const auto q=queries(); assert(!q.empty()); Packet p;
  p.type='D'; p.session=q.back().session; p.seq=q.back().seq;
  assert(encodeScene(scene,p.payload,sizeof(p.payload))); return p;
}
static void wire(const Packet& packet) {
  char frame[kMaxFrame]; assert(encodePacket(packet,frame,sizeof(frame)));
  camera.input=frame; camera.cursor=0;
  while (camera.available()) loop();
}
static void waitRun(RunState wanted) {
  const uint32_t start=millis();
  while (runState!=wanted) {
    assert(runState!=RunState::Fault && runState!=RunState::Stopped);
    assert(uint32_t(millis()-start)<120000); advance(20);
  }
}
static JointPose persisted() {
  app::PoseStore probe; JointPose result={}; bool empty=true;
  assert(probe.begin() && probe.load(result,empty) && !empty); return result;
}
static void seedStorage(const JointPose& pose) {
  app::PoseStore probe; JointPose previous={}; bool empty;
  assert(probe.begin() && probe.load(previous,empty) && probe.save(pose));
}
static void beginSort() { send("SORT"); waitRun(RunState::Sorting); assert(sortState("WAIT_CAMERA")); }
static void enableLevel(bool enabled,bool interrupt=true) {
  fakehw::levels[app::ENABLE_PIN]=enabled?HIGH:LOW;
  if (!enabled && interrupt && fakehw::interrupts[app::ENABLE_PIN]) fakehw::interrupts[app::ENABLE_PIN]();
}
static void press(bool pressed,uint32_t duration=60) {
  fakehw::levels[app::SORT_BUTTON_PIN]=pressed?LOW:HIGH; advance(duration);
}
static void completeEmpty() {
  waitRun(RunState::Sorting); Scene empty=parts(); empty.count=0; wire(response(empty));
  assert(runState==RunState::Ready && motion.outputsOn());
}
static void assertNoRestart() {
  const auto writes=fakehw::writeCalls, attaches=fakehw::attachCalls;
  const size_t scans=queries().size();
  enableLevel(true); fakehw::failWrite=-1; fakehw::failAttach=-1;
  advance(1200);
  assert(!motion.outputsOn() && !sorterController.busy());
  assert(fakehw::writeCalls==writes && fakehw::attachCalls==attaches && queries().size()==scans);
}

int main() {
  // First startup issues only PARK commands, without beginning a camera batch.
  reboot();
  assert(runState==RunState::Ready && storageReady && motion.outputsOn());
  assert(same(motion.commandedPose(),arm_map::PARK) && same(persisted(),arm_map::PARK));
  assert(fakehw::attachCalls==4 && fakehw::writeCalls==4 && queries().empty());
  assert(fakehw::modes[app::ENABLE_PIN]==INPUT_PULLDOWN && fakehw::modes[app::SORT_BUTTON_PIN]==INPUT_PULLUP);
  assert(camera.initialized && camera.rx==16 && camera.tx==17 && camera.baud==115200);
  const auto firstWrites=fakenvs::puts; advance(3000);
  assert(fakenvs::puts==firstWrites && queries().empty()); // Stationary poses do not wear flash.
  advance(31UL*60UL*1000UL);
  assert(runState==RunState::Ready && motion.outputsOn() && fakenvs::puts==firstWrites);

  // A saved closed-gripper pose attaches unchanged. An explicit SORT alone
  // traverses HUB, opens at PARK, settles, and only then asks for a frame.
  const JointPose saved={{1800,1900,950,1250}};
  seedStorage(saved); reboot(false);
  assert(same(motion.commandedPose(),saved) && queries().empty() && fakehw::writeCalls==4);
  send("SORT"); assert(runState==RunState::Hub && queries().empty());
  bool sawHub=false, sawPark=false;
  const uint32_t began=millis();
  while (runState!=RunState::Sorting) {
    assert(uint32_t(millis()-began)<10000); advance(20);
    if (runState==RunState::Hub) assert(motion.commandedPose().us[3]==1250);
    if (runState==RunState::Park && !sawHub) {
      JointPose hub=arm_map::HUB; hub.us[3]=1250;
      assert(same(motion.commandedPose(),hub) && same(persisted(),hub)); sawHub=true;
    }
    if (runState==RunState::Settling) {
      sawPark=true; assert(same(motion.commandedPose(),arm_map::PARK)); assert(queries().empty());
      assert(same(persisted(),arm_map::PARK));
    }
  }
  assert(sawHub && sawPark && queries().size()==1); completeEmpty();

  // ENABLE absent at startup defers attachment. STOP cancels that pending startup.
  reboot(true,false); assert(runState==RunState::WaitEnable && !fakehw::attachCalls);
  advance(500); enableLevel(true); advance(20);
  assert(runState==RunState::Ready && motion.outputsOn() && queries().empty());
  reboot(true,false); send("STOP"); enableLevel(true); assertNoRestart();
  send("SORT"); completeEmpty();

  // A held boot button and short bounce never start a batch. Presses while busy
  // are consumed, including one held through completion; nothing is queued.
  reboot(true,true,true); advance(3000); assert(queries().empty());
  press(false); press(true,10); press(false,60); assert(queries().empty());
  press(true); assert(runState==RunState::Settling); completeEmpty();
  advance(1200); assert(queries().size()==1);
  press(false); press(true); waitRun(RunState::Sorting);
  press(false); press(true); send("SORT"); assert(output("request not queued"));
  completeEmpty(); advance(1500); assert(queries().size()==2 && runState==RunState::Ready);
  press(false); press(true); completeEmpty(); assert(queries().size()==3);

  // Whole-sketch transfers for both colour orders, every planned endpoint,
  // fresh PARK frames and counters. Reached and moving poses are checkpointed.
  for (bool blueFirst : {false,true}) for (uint32_t now : {0U,UINT32_MAX-4000U}) {
    reboot(true,true,false,now); beginSort(); Scene scene=parts(blueFirst);
    for (unsigned pick=0;pick<2;++pick) {
      uint8_t counts[2]={}; const auto map=arm_map::calibration(); camera_sort::Plan expected={};
      assert(!camera_sort::inspectSideScene(scene,map,counts));
      assert(camera_sort::makeSidePlan(scene.objects[0],counts,map,motionSettings,expected));
      wire(response(scene)); unsigned endpoint=0; const uint32_t beganPick=millis();
      size_t lastSave=fakenvs::writes.size(); uint32_t lastSavedTime=millis();
      while (!sortState("VERIFY_CAMERA")) {
        const JointPose previous=motion.commandedPose(); advance(20);
        assert(runState==RunState::Sorting && uint32_t(millis()-beganPick)<100000);
        const unsigned maxStep=(unsigned(app::RATE_US_PER_SECOND)*20U+999U)/1000U;
        for (unsigned j=0;j<4;++j)
          assert(unsigned(std::abs(int(previous.us[j])-int(motion.commandedPose().us[j])))<=maxStep);
        if (endpoint<14 && same(motion.commandedPose(),expected.poses[endpoint])) ++endpoint;
        if (motion.state()==JogState::Holding) {
          assert(same(persisted(),motion.commandedPose())); lastSavedTime=millis();
        }
        if (fakenvs::writes.size()!=lastSave) { lastSave=fakenvs::writes.size(); lastSavedTime=millis(); }
        if (motion.state()==JogState::Jogging) assert(uint32_t(millis()-lastSavedTime)<=560);
      }
      assert(endpoint==14 && same(persisted(),arm_map::PARK));
      if (scene.count==2) scene.objects[0]=scene.objects[1]; --scene.count;
    }
    wire(response(scene));
    assert(runState==RunState::Ready && motion.outputsOn());
    assert(sorterController.sortedRed()==1 && sorterController.sortedBlue()==1 && sorterController.sortAttempts()==2);
    assert(queries().size()==3 && output("distance_mm=25") && output("direction=+Y"));
    assert(output("name=SIDE_HIGH") && output("name=SIDE_LOW") && output("name=ENTER_Y"));
  }

  // Stops and faults save the last completed command, disable all outputs and
  // cannot restart on stale packets, elapsed time or restored ENABLE alone.
  for (unsigned cause=0;cause<5;++cause) {
    reboot(); beginSort(); wire(response(parts()));
    const JointPose beforeMove=motion.commandedPose(); const uint32_t moveStarted=millis();
    while (same(beforeMove,motion.commandedPose())) {
      assert(uint32_t(millis()-moveStarted)<1000); advance(20);
    }
    assert(motion.state()==JogState::Jogging);
    const Packet stale=response(parts());
    if (cause==0) send("STOP");
    if (cause==1) send("ESTOP");
    if (cause==2) { enableLevel(false); advance(20); }
    if (cause==3) { fakehw::failWrite=25; advance(20); }
    if (cause==4) { fakehw::now+=300; loop(); }
    assert(!motion.outputsOn() && !sorterController.busy());
    const JointPose stopped=lastCommand;
    assert(same(persisted(),stopped));
    assert(!fakenvs::writes.empty() && !fakenvs::writes.back().outputsOn);
    wire(stale); assertNoRestart();
    send("SORT"); assert(motion.outputsOn()); completeEmpty();
    reboot(false); assert(same(motion.commandedPose(),arm_map::PARK) && queries().empty());
  }
  // A transient falling edge remains latched even if GPIO is already high.
  reboot(); enableLevel(false); enableLevel(true); advance(20);
  assert(!motion.outputsOn()); assertNoRestart(); send("SORT"); completeEmpty();

  // Losing power during a move restores the last checkpoint without a batch.
  reboot(); beginSort(); wire(response(parts())); advance(1780);
  const JointPose checkpoint=persisted(); assert(!same(checkpoint,arm_map::PARK));
  reboot(false); assert(same(motion.commandedPose(),checkpoint) && queries().empty());

  // Latest-record corruption or a torn write falls back to the other good
  // record. Two invalid records, or an incompatible profile, must not move.
  reboot(); seedStorage(saved); const auto good=fakenvs::records;
  fakenvs::records[fakenvs::writes.back().key][0]^=1;
  reboot(false); assert(same(motion.commandedPose(),arm_map::PARK));
  fakenvs::records=good;
  for (auto& record : fakenvs::records) record.second[0]^=1;
  reboot(false); assert(runState==RunState::Fault && !fakehw::attachCalls && !storageReady);
  send("SORT"); assert(!fakehw::attachCalls);
  fakenvs::clear(); fakenvs::records["sorter-pose/pose-a"]={};
  reboot(false); assert(runState==RunState::Fault && !fakehw::attachCalls && !storageReady);
  // Wrong-type NVS keys also report zero blob length but remain present.
  fakenvs::clear(); fakenvs::nonBlobKeys.insert("sorter-pose/pose-b");
  reboot(false); assert(runState==RunState::Fault && !fakehw::attachCalls && !storageReady);
  // Valid CRC alone is insufficient: incompatible profile/version and servo
  // commands outside working limits all block restoration.
  for (unsigned incompatible=0; incompatible<3; ++incompatible) {
    fakenvs::clear(); seedStorage(saved);
    auto& bytes=fakenvs::records.begin()->second;
    if (incompatible==0) bytes[4]^=1;
    if (incompatible==1) bytes[12]=2;
    if (incompatible==2) { bytes[14]=0; bytes[15]=0; }
    const uint16_t check=crc16(bytes.data(),22); bytes[22]=uint8_t(check); bytes[23]=uint8_t(check>>8);
    reboot(false); assert(runState==RunState::Fault && !fakehw::attachCalls && !storageReady);
  }
  fakenvs::clear(); seedStorage(saved); fakenvs::partialPut=7;
  { app::PoseStore store; JointPose p={}; bool empty;
    assert(store.begin() && store.load(p,empty)); assert(!store.save(arm_map::PARK)); }
  fakenvs::partialPut=0; reboot(false);
  assert(same(motion.commandedPose(),saved));

  // Flash failures are surfaced before startup or stop a running move.
  for (unsigned failure=0;failure<4;++failure) {
    fakenvs::clear();
    if (failure==0) fakenvs::failBegin=true;
    if (failure==1) fakenvs::failPut=true;
    if (failure==2) fakenvs::corruptRead=true;
    if (failure==3) { seedStorage(saved); fakenvs::failRead=true; }
    reboot(false); assert(runState==RunState::Fault && !motion.outputsOn() && !fakehw::attachCalls);
  }
  reboot(); beginSort(); wire(response(parts())); advance(100);
  fakenvs::failPut=true; advance(500);
  assert(runState==RunState::Fault && !storageReady && !motion.outputsOn());
  assert(!std::strcmp(appError,"POSE_SAVE_FAILED"));
  fakenvs::failPut=false; assertNoRestart(); send("SORT"); assert(!motion.outputsOn());

  // A slower valid motion configuration lets the real eight-part route reach
  // the batch deadline while still moving. The coordinator's own disarm must
  // persist the final command even though it happens after the checkpoint step.
  reboot(); motionSettings.rateUsPerSecond=50; beginSort();
  Scene longBatch=parts(); longBatch.count=8;
  for (unsigned i=0;i<8;++i)
    longBatch.objects[i]={uint8_t((i%4)<2?3:1),int32_t(200+350*(i%4)),int32_t(i<4?500:800),400};
  wire(response(longBatch));
  const uint32_t longStarted=millis();
  while (runState==RunState::Sorting) {
    advance(20);
    assert(uint32_t(millis()-longStarted)<920000);
    if (sortState("VERIFY_CAMERA")) {
      for (unsigned i=1;i<longBatch.count;++i) longBatch.objects[i-1]=longBatch.objects[i];
      --longBatch.count; wire(response(longBatch));
    }
  }
  assert(runState==RunState::Fault && !motion.outputsOn());
  assert(!std::strcmp(appError,"BATCH_TIME_LIMIT"));
  assert(same(persisted(),motion.commandedPose()) && same(persisted(),lastCommand));
  assertNoRestart();

  // Damaged CRC/stale session packets never reach a scene transition.
  reboot(); beginSort(); Packet packet=response(parts());
  char damaged[kMaxFrame]; assert(encodePacket(packet,damaged,sizeof(damaged)));
  damaged[2]=damaged[2]=='1'?'2':'1'; camera.input=damaged; camera.cursor=0;
  while (camera.available()) loop();
  assert(sortState("WAIT_CAMERA") && cameraParser.errors()==1);
  ++packet.session; wire(packet); assert(sortState("WAIT_CAMERA"));
  advance(6100); assert(runState==RunState::Fault && sortState("FAILED"));
  assert(!std::strcmp(appError,"CAMERA_TIMEOUT") && same(persisted(),arm_map::PARK));

  std::puts("SORT whole sketch: startup/restore, button, both colours, persistence, faults and no autoresume PASS (fake hardware only)");
}
