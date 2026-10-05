#include "MotionCore.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <vector>

using namespace sorter;
struct Request { uint32_t session, seq, calibration; };
struct Write { uint32_t at; JointPose pose; };
struct FakeIO : MotionIO {
  uint32_t now = 0;
  bool enableOk = true, writeOk = true, requestOk = true;
  bool physicalEnable = true, physicalSupply = true;
  unsigned enables = 0, detaches = 0;
  std::vector<Request> requests;
  std::vector<Write> writes;
  bool enableAt(const JointPose& p) override { ++enables; writes.push_back({now,p}); return enableOk; }
  bool writePose(const JointPose& p) override { writes.push_back({now,p}); return writeOk; }
  void detachOutputs() override { ++detaches; }
  bool hardwareEnablePresent() const override { return physicalEnable; }
  bool servoSupplyPresent() const override { return physicalSupply; }
  bool requestScene(uint32_t session, uint32_t seq, uint32_t calibration) override {
    requests.push_back({session,seq,calibration}); return requestOk;
  }
};
struct Rig {
  MotionCalibration c;
  MotionSettings s;
  FakeIO io;
  MotionController m;
  uint32_t now;
  Rig(uint32_t initial = 0) : c(exampleVirtualCalibration()), s(defaultMotionSettings()),
    io(), m(c,s,io), now(initial) { io.now = now; }
  void tick(bool enable = true, bool power = true, uint32_t dt = 20) {
    now += dt; io.now = now; m.tick(now,enable,power);
  }
  void arm() {
    m.confirmOperatorPark(); assert(m.arm(now,12345,true,true));
  }
  void until(MotionState target, uint32_t limit = 200000) {
    uint32_t start = now;
    while (m.state() != target && uint32_t(now-start) < limit && m.state() != MotionState::Fault) tick();
    if (m.state() != target) fprintf(stderr,"wanted %s got %s (%s)\n",motionStateName(target),
      motionStateName(m.state()),motionErrorName(m.error()));
    assert(m.state() == target);
  }
  bool scene(const Scene& scene) { return m.receiveScene(m.session(),m.sequence(),scene,now); }
};
Scene one(uint8_t cls = 1, int32_t x = 0, int32_t y = 1000, uint32_t boot = 456) {
  Scene s = {}; s.cameraBoot = boot; s.calibrationId = 1; s.count = 1;
  s.objects[0].classId = cls; s.objects[0].x10 = x; s.objects[0].y10 = y; s.objects[0].pixels = 250;
  return s;
}
Scene empty(uint32_t boot = 456) {
  Scene s = {}; s.cameraBoot = boot; s.calibrationId = 1; return s;
}
void fullyMeasured(MotionCalibration& c) {
  c.geometryConfirmed = c.routesVerified = c.parkSupported = true;
  for (unsigned i = 0; i < 4; ++i) c.grid[i].measuredLow = c.grid[i].measuredHigh = true;
  c.parkOpen.measured = c.hubOpen.measured = true;
  for (unsigned i = 0; i < kColorCount; ++i) c.binHighOpen[i].measured = c.binDropOpen[i].measured = true;
}

void testCalibration() {
  MotionCalibration c = exampleVirtualCalibration();
  assert(calibrationValid(c,false)); assert(!calibrationValid(c,true));
  CalibratedPose p = {};
  assert(interpolateGrid(c,0,1000,false,p)); assert(p.joints.us[0] == 1500);
  assert(p.joints.us[1] == 1650); assert(p.joints.us[2] == 1475); assert(!p.measured);
  assert(!interpolateGrid(c,c.pickMinX10-1,1000,false,p));
  assert(!interpolateGrid(c,INT32_MAX,INT32_MIN,true,p));
  for (int32_t x = c.pickMinX10; x <= c.pickMaxX10; x += 25)
    for (int32_t y = c.pickMinY10; y <= c.pickMaxY10; y += 25)
      for (unsigned high = 0; high < 2; ++high) {
        assert(interpolateGrid(c,x,y,high,p)); assert(poseWithinLimits(p.joints,c));
        assert(pointWithinBounds(p.tip,c));
      }
  fullyMeasured(c); assert(calibrationValid(c,true));
  c.grid[2].measuredHigh = false; assert(!calibrationValid(c,true));
  c.grid[2].measuredHigh = true; c.lowZ10 = c.minZ10-1; assert(!calibrationValid(c,true));
  c = exampleVirtualCalibration(); c.grid[1].lowOpen.us[0] = 2500;
  assert(!calibrationValid(c,false));
  c = exampleVirtualCalibration(); c.pickMaxX10 = c.pickMinX10;
  assert(!calibrationValid(c,false));
}
void testNoAutomaticOutputs() {
  Rig r; for (unsigned i = 0; i < 100; ++i) r.tick();
  assert(r.io.enables == 0 && r.io.writes.empty() && r.io.requests.empty());
  assert(!r.m.arm(r.now,123,true,true)); assert(r.m.error() == MotionError::ParkNotConfirmed);
  r.m.confirmOperatorPark(); assert(!r.m.arm(r.now,123,false,true));
  assert(r.m.error() == MotionError::EnableAbsent);
  assert(!r.m.arm(r.now,123,true,true)); // Previous acknowledgment was consumed.
  r.arm(); assert(r.io.enables == 1);
}
void testHardwareGates() {
  Rig r; r.s.virtualMode = false; r.m.confirmOperatorPark();
  assert(!r.m.arm(r.now,123,true,true)); assert(r.m.error() == MotionError::BadConfig);
  r.s.outputsEnabled = r.s.pinoutConfirmed = true; fullyMeasured(r.c);
  r.arm(); r.tick(true,false); assert(r.m.state() == MotionState::Disarmed);
  assert(r.m.error() == MotionError::PowerLost && !r.m.outputsOn());
  for (unsigned i = 0; i < 100; ++i) r.tick();
  assert(r.io.enables == 1); assert(!r.m.arm(r.now,55,true,true));
  assert(r.m.error() == MotionError::ParkNotConfirmed);
}
void testRetryAndTimeout() {
  Rig r; r.arm(); r.until(MotionState::WaitScene);
  const Request request = r.io.requests[0];
  while (r.m.state() != MotionState::Fault) r.tick();
  assert(r.m.error() == MotionError::CameraTimeout); assert(r.io.requests.size() == 3);
  for (const auto& q : r.io.requests) assert(q.session == request.session && q.seq == request.seq);
  assert(!r.m.outputsOn());
}
void testLateReplyIsNotConsumed() {
  Rig r; r.arm(); r.until(MotionState::WaitScene);
  const uint32_t requestedAt = r.now;
  // Direct injection deliberately omits the timer tick normally run by firmware.
  assert(!r.m.receiveScene(r.m.session(),r.m.sequence(),one(),requestedAt+r.s.sceneTimeoutMs));
  assert(r.m.state() == MotionState::WaitScene && r.m.attempts() == 0);
  while (uint32_t(r.now-requestedAt) < r.s.sceneTimeoutMs) r.tick();
  assert(r.io.requests.size() == 2);
  assert(r.io.requests[0].seq == r.io.requests[1].seq);
  assert(r.scene(one()));
}
void testOneCompleteCycleAndBounds() {
  Rig r; r.arm(); r.until(MotionState::WaitScene); const uint32_t seq = r.m.sequence();
  assert(!r.m.receiveScene(r.m.session()+1,seq,one(),r.now));
  assert(!r.m.receiveScene(r.m.session(),seq+1,one(),r.now));
  assert(r.scene(one())); assert(r.m.attempts() == 1);
  assert(!r.scene(one())); // D result is consumed exactly once.
  r.until(MotionState::WaitScene);
  assert(r.m.sequence() == seq+1); assert(r.m.attempts() == 1);
  assert(r.scene(empty())); assert(r.m.state() == MotionState::Done);
  assert(r.m.clearedTargets() == 1 && !r.m.outputsOn());
  assert(r.io.requests.size() == 2);
  for (size_t i = 0; i < r.io.writes.size(); ++i) {
    assert(poseWithinLimits(r.io.writes[i].pose,r.c));
    if (!i) continue;
    const uint32_t dt = uint32_t(r.io.writes[i].at-r.io.writes[i-1].at);
    for (unsigned j = 0; j < 4; ++j) {
      int delta = int(r.io.writes[i].pose.us[j])-r.io.writes[i-1].pose.us[j];
      if (delta < 0) delta = -delta;
      // Allow one integer-microsecond rounding quantum.
      assert(delta <= int(ceil(r.c.maxRateUsPerSecond[j]*double(dt)/1000.0))+1);
    }
  }
}
void testAllBinsAndGridCorners() {
  for (uint8_t cls : kColorClassIds) for (unsigned corner = 0; corner < 4; ++corner) {
    Rig r; r.arm(); r.until(MotionState::WaitScene);
    assert(r.scene(one(cls,(corner&1)?300:-300,(corner&2)?1200:800)));
    r.until(MotionState::WaitScene);
    bool dropFound = false;
    for (const auto& w : r.io.writes) {
      const JointPose& p = r.c.binDropOpen[classBinIndex(cls)].joints;
      if (memcmp(&p,&w.pose,sizeof(p)) == 0) dropFound = true;
    }
    assert(dropFound); assert(r.scene(empty())); assert(r.m.state() == MotionState::Done);
  }
}
void testCameraRestartAndMismatches() {
  Rig r; r.arm(); r.until(MotionState::WaitScene); assert(r.scene(one()));
  r.until(MotionState::WaitScene);
  assert(!r.m.receiveScene(r.m.session(),r.m.sequence()-1,one(1,0,1000,999),r.now));
  assert(r.m.state() == MotionState::WaitScene);
  assert(!r.scene(empty(999))); assert(r.m.error() == MotionError::CameraRestart);
  Rig x; x.arm(); x.until(MotionState::WaitScene);
  Scene s = one(); s.calibrationId = 2;
  assert(!x.scene(s)); assert(x.m.error() == MotionError::CalibrationMismatch);
}
void testNoProgressAndAttemptLimit() {
  Rig r; r.arm(); r.until(MotionState::WaitScene); assert(r.scene(one()));
  r.until(MotionState::WaitScene); assert(r.scene(one()));
  r.until(MotionState::WaitScene); assert(!r.scene(one()));
  assert(r.m.error() == MotionError::NoProgress && r.m.attempts() == 2);
  Rig x; x.s.maxAttempts = 1; x.arm(); x.until(MotionState::WaitScene); assert(x.scene(one()));
  x.until(MotionState::WaitScene); assert(!x.scene(one(3,200,1100)));
  assert(x.m.error() == MotionError::BatchLimit);
}
void testPauseAndStaleCapture() {
  Rig r; r.arm(); r.until(MotionState::WaitScene); const uint32_t previous = r.m.sequence();
  r.m.pause(r.now); assert(!r.scene(one()));
  for (unsigned i = 0; i < 25; ++i) r.tick();
  r.m.resume(r.now); assert(r.m.sequence() == previous+1);
  assert(!r.m.receiveScene(r.m.session(),previous,one(),r.now));
  assert(r.scene(one()));
  for (unsigned i = 0; i < 15; ++i) r.tick();
  r.m.pause(r.now); const JointPose held = r.m.commandedPose();
  const size_t writes = r.io.writes.size();
  for (unsigned i = 0; i < 50; ++i) r.tick();
  assert(r.io.writes.size() == writes);
  assert(memcmp(&held,&r.m.commandedPose(),sizeof(held)) == 0 && r.m.outputsOn());
  r.m.resume(r.now); r.until(MotionState::WaitScene);
  assert(r.scene(empty())); assert(r.m.state() == MotionState::Done);
}
void testEstopAndPauseLimit() {
  Rig r; r.arm(); r.until(MotionState::WaitScene); assert(r.scene(one()));
  r.m.emergencyStop(); assert(r.m.state() == MotionState::Fault && !r.m.outputsOn());
  r.m.confirmOperatorPark(); assert(!r.m.arm(r.now,11,true,true));
  r.m.resetFault(); assert(r.m.state() == MotionState::Disarmed);
  assert(!r.m.arm(r.now,11,true,true));
  Rig x; x.s.maxPauseMs = 500; x.arm(); x.until(MotionState::WaitScene); x.m.pause(x.now);
  for (unsigned i = 0; i < 30; ++i) x.tick();
  assert(x.m.error() == MotionError::PauseLimit && !x.m.outputsOn());
}
void testFailureAndWatchdog() {
  Rig r; r.io.enableOk = false; r.m.confirmOperatorPark();
  assert(!r.m.arm(r.now,1,true,true)); assert(r.io.detaches > 0 && !r.m.outputsOn());
  Rig x; x.arm(); x.io.requestOk = false;
  for (unsigned i = 0; i < 50; ++i) x.tick();
  assert(x.m.error() == MotionError::OutputFailure);
  Rig w; w.arm(); w.until(MotionState::WaitScene); assert(w.scene(one()));
  w.io.writeOk = false; w.tick(); assert(w.m.error() == MotionError::OutputFailure);
  Rig race; race.arm(); race.until(MotionState::WaitScene); assert(race.scene(one()));
  race.io.writeOk = false; race.io.physicalSupply = false;
  race.tick(true,true); // Input loss happened after the caller's input snapshot.
  assert(race.m.state() == MotionState::Disarmed && race.m.error() == MotionError::PowerLost);
  Rig y; y.arm(); y.tick(true,true,251); assert(y.m.error() == MotionError::LoopLate);
  Rig z; z.s.maxBatchMs = 1000; z.arm();
  for (unsigned i = 0; i < 55; ++i) z.tick();
  assert(z.m.error() == MotionError::BatchLimit);
}
void testMalformedAndUnreachable() {
  Rig r; r.arm(); r.until(MotionState::WaitScene); Scene s = one(); s.count = 9;
  assert(!r.scene(s)); assert(r.m.error() == MotionError::MalformedScene);
  Rig x; x.arm(); x.until(MotionState::WaitScene); assert(!x.scene(one(1,INT32_MAX,INT32_MIN)));
  assert(x.m.error() == MotionError::NoReachableObject);
  Rig y; y.arm(); y.until(MotionState::WaitScene); s = one(7);
  assert(!y.scene(s)); assert(y.m.error() == MotionError::MalformedScene);
  Rig green; green.arm(); green.until(MotionState::WaitScene);
  const size_t greenWrites = green.io.writes.size();
  assert(!green.scene(one(2)));
  assert(green.m.error() == MotionError::MalformedScene && green.m.attempts() == 0);
  assert(green.io.writes.size() == greenWrites);
  Rig z; z.arm(); z.until(MotionState::WaitScene);
  assert(!z.m.receiveError(z.m.session()+1,z.m.sequence()));
  assert(z.m.receiveError(z.m.session(),z.m.sequence())); assert(z.m.error() == MotionError::CameraError);
}
void testWrapAround() {
  Rig r(UINT32_MAX-400); r.arm(); r.until(MotionState::WaitScene);
  assert(r.scene(one())); r.until(MotionState::WaitScene); assert(r.scene(empty()));
  assert(r.m.state() == MotionState::Done);
}
int main() {
  assert(kColorCount == 2 && classBinIndex(1) == 0 && classBinIndex(3) == 1);
  assert(!isSupportedClass(2) && classBinIndex(2) == 255);
  testCalibration(); testNoAutomaticOutputs(); testHardwareGates(); testRetryAndTimeout();
  testLateReplyIsNotConsumed();
  testOneCompleteCycleAndBounds(); testAllBinsAndGridCorners(); testCameraRestartAndMismatches();
  testNoProgressAndAttemptLimit(); testPauseAndStaleCapture(); testEstopAndPauseLimit();
  testFailureAndWatchdog(); testMalformedAndUnreachable(); testWrapAround();
  puts("motion tests: PASS (14 suites; protocol ordering, routes, limits and stop semantics)");
  return 0;
}
