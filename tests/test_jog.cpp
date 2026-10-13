#include <ManualJogCore.h>
#include <cassert>
#include <cstdio>
#include <vector>

using namespace sorter;
struct FakeJogIO : JogIO {
  unsigned enables = 0, detaches = 0;
  bool enableOkay = true, writeOkay = true;
  std::vector<JointPose> writes;
  bool enableAt(const JointPose& p) override { ++enables; writes.push_back(p); return enableOkay; }
  bool writePose(const JointPose& p) override { writes.push_back(p); return writeOkay; }
  void detachOutputs() override { ++detaches; }
};
JogSettings enabledSettings() {
  JogSettings s = defaultJogSettings();
  s.outputsEnabled = s.pinoutConfirmed = s.limitsConfirmed = true;
  return s;
}
JogSettings patientSettings() {
  JogSettings s = enabledSettings();
  s.maxStepUs = 100;
  s.holdLeaseMs = 300000;
  s.maxSessionMs = 1800000;
  return s;
}
JogSettings travelSettings() {
  JogSettings s = patientSettings();
  for (uint8_t j = 0; j < 4; ++j) { s.minUs[j] = 500; s.maxUs[j] = 2500; }
  return s;
}
const JointPose seed = {{1500,1500,1500,1500}};
void prepare(ManualJogController& c, uint32_t now = 0) {
  assert(c.seed(seed)); assert(c.confirmSupported()); assert(c.arm(now, true, true));
}
void defaultsAndExplicitSeed() {
  JogSettings s = defaultJogSettings(); FakeJogIO io; ManualJogController c(s, io);
  c.tick(100, true, true); assert(io.enables == 0 && io.writes.empty());
  assert(c.seed(seed) && c.confirmSupported()); assert(!c.arm(100, true, true));
  assert(c.error() == JogError::Config && io.enables == 0);
  s = enabledSettings();
  assert(!c.arm(100, true, true)); // Previous failed ARM consumed support confirmation.
  assert(c.error() == JogError::Support);
  assert(c.confirmSupported() && c.arm(100, true, true));
  c.disarm(); assert(!c.seedPresent() && !c.outputsOn());
  assert(!c.arm(120, true, true)); assert(c.error() == JogError::Seed);
  JointPose out = seed; out.us[2] = 2000;
  assert(!c.seed(out)); assert(!c.confirmSupported());
  assert(c.seed(seed) && c.confirmSupported());
  const unsigned enables = io.enables;
  assert(!c.arm(120, false, true) && c.error() == JogError::Permit);
  c.tick(140, true, true); assert(io.enables == enables && !c.outputsOn());
  assert(!c.arm(140, true, true) && c.error() == JogError::Support);
}
void boundedSingleJointAndNoQueue() {
  JogSettings s = enabledSettings(); FakeJogIO io; ManualJogController c(s, io); prepare(c);
  assert(!c.jog(4, 1, 0, true, true)); assert(!c.jog(0, 26, 0, true, true));
  assert(!c.jog(0, -26, 0, true, true)); assert(!c.jog(0, 0, 0, true, true));
  assert(c.jog(2, -25, 0, true, true));
  assert(!c.jog(0, 10, 0, true, true)); // No queued second move.
  assert(!c.seed(seed));
  for (uint32_t t = 20; t <= 300; t += 20) c.tick(t, true, true);
  assert(c.state() == JogState::Holding && c.commandedPose().us[2] == 1475);
  for (size_t i = 1; i < io.writes.size(); ++i) {
    for (uint8_t j = 0; j < 4; ++j) if (j != 2) assert(io.writes[i].us[j] == 1500);
    const int delta = int(io.writes[i].us[2]) - io.writes[i-1].us[2];
    assert(delta >= -2 && delta <= 0);
  }
  c.disarm(); JointPose limit = seed; limit.us[0] = 1600;
  assert(c.seed(limit) && c.confirmSupported() && c.arm(300, true, true));
  assert(!c.jog(0, 1, 300, true, true)); assert(c.commandedPose().us[0] == 1600);
}
void leaseAndSessionAreFinite() {
  JogSettings s = enabledSettings(); s.holdLeaseMs = 1000; s.maxSessionMs = 2000;
  FakeJogIO io; ManualJogController c(s, io); prepare(c);
  for (uint32_t t = 20; t <= 1000; t += 20) c.tick(t, true, true);
  assert(c.state() == JogState::Disarmed && c.error() == JogError::LeaseExpired);
  assert(io.detaches == 1 && !c.seedPresent());
  assert(!c.hold(1000, true, true)); // Expired lease cannot be renewed retroactively.
  prepare(c, 1000);
  for (uint32_t t = 1020; t <= 3000; t += 20) {
    c.tick(t, true, true);
    if (t % 500 == 0 && t < 3000) assert(c.hold(t, true, true));
  }
  assert(c.state() == JogState::Fault && c.error() == JogError::SessionExpired);
  assert(!c.outputsOn()); c.reset(); assert(c.state() == JogState::Disarmed);
}
void physicalLossAndOutputs() {
  for (uint8_t loss = 0; loss < 2; ++loss) {
    JogSettings s = enabledSettings(); FakeJogIO io; ManualJogController c(s, io); prepare(c);
    assert(c.jog(0, 25, 0, true, true)); const size_t writes = io.writes.size();
    c.tick(20, loss != 0, loss == 0);
    assert(c.state() == JogState::Fault && io.detaches == 1 && io.writes.size() == writes);
    c.tick(40, true, true); assert(!c.outputsOn()); assert(!c.arm(40, true, true));
    c.reset(); assert(!c.seedPresent());
  }
  JogSettings s = enabledSettings(); FakeJogIO io; ManualJogController c(s, io);
  io.enableOkay = false; assert(c.seed(seed) && c.confirmSupported());
  assert(!c.arm(0, true, true)); assert(io.detaches == 1 && !c.outputsOn());
  c.reset(); io.enableOkay = true; prepare(c); io.writeOkay = false;
  assert(c.jog(0, 10, 0, true, true)); c.tick(20, true, true);
  assert(c.error() == JogError::Output && io.detaches == 2);
}
void timingAndEstop() {
  JogSettings s = enabledSettings(); FakeJogIO io; ManualJogController c(s, io); prepare(c);
  c.tick(251, true, true); assert(c.error() == JogError::LoopLate && !c.outputsOn());
  c.reset(); const uint32_t start = UINT32_MAX - 49; prepare(c, start);
  assert(c.jog(0, 10, start, true, true));
  for (uint32_t elapsed = 20; elapsed <= 120; elapsed += 20) c.tick(start + elapsed, true, true);
  assert(c.state() == JogState::Holding && c.commandedPose().us[0] == 1510);
  c.estop(); assert(c.state() == JogState::Fault && !c.outputsOn());
  c.reset(); assert(c.state() == JogState::Disarmed && !c.seedPresent());
}
void extendedSettingsRemainBounded() {
  JogSettings s = patientSettings(); FakeJogIO io; ManualJogController c(s, io);
  assert(c.configurationValid());
  s.maxStepUs = 101; assert(!c.configurationValid());
  s.maxStepUs = 0; assert(!c.configurationValid());
  s.maxStepUs = 100;
  s.holdLeaseMs = 300001; assert(!c.configurationValid());
  s.holdLeaseMs = 999; assert(!c.configurationValid());
  s.holdLeaseMs = 300000;
  s.maxSessionMs = 1800001; assert(!c.configurationValid());
  s.maxSessionMs = 299999; assert(!c.configurationValid());
  s.maxSessionMs = 1800000;
  s.rateUsPerSecond = 1000; assert(c.configurationValid());
  s.rateUsPerSecond = 1001; assert(!c.configurationValid());
  s.rateUsPerSecond = 49; assert(!c.configurationValid());
  s.rateUsPerSecond = 100;
  s.maxLoopGapMs = 251; assert(!c.configurationValid());
  s.maxLoopGapMs = 19; assert(!c.configurationValid());
  s.maxLoopGapMs = 250;
  assert(c.configurationValid() && io.enables == 0 && io.writes.empty());
}
void largerStepsStaySmoothAndSingleJoint() {
  JogSettings s = patientSettings(); FakeJogIO io; ManualJogController c(s, io); prepare(c);
  assert(!c.jog(2, 101, 0, true, true));
  assert(!c.jog(2, -101, 0, true, true));
  assert(c.jog(2, 100, 0, true, true));
  assert(!c.jog(0, 25, 0, true, true)); // No queued motion on another joint.
  assert(c.error() == JogError::Busy);
  for (uint32_t t = 20; t < 1000; t += 20) {
    c.tick(t, true, true);
    assert(c.state() == JogState::Jogging);
  }
  c.tick(1000, true, true);
  assert(c.state() == JogState::Holding && c.error() == JogError::None);
  assert(c.commandedPose().us[2] == 1600 && io.writes.size() == 51);
  for (size_t i = 1; i < io.writes.size(); ++i) {
    for (uint8_t j = 0; j < 4; ++j) if (j != 2) assert(io.writes[i].us[j] == 1500);
    assert(io.writes[i].us[2] - io.writes[i-1].us[2] == 2);
  }
  assert(c.jog(2, -100, 1000, true, true));
  for (uint32_t t = 1020; t <= 2000; t += 20) c.tick(t, true, true);
  assert(c.state() == JogState::Holding && c.commandedPose().us[2] == 1500);
  for (size_t i = 51; i < io.writes.size(); ++i) {
    for (uint8_t j = 0; j < 4; ++j) if (j != 2) assert(io.writes[i].us[j] == 1500);
    assert(int(io.writes[i].us[2]) - io.writes[i-1].us[2] == -2);
  }
}
void largerPermissionDoesNotExpandJointLimits() {
  JogSettings s = patientSettings();
  for (uint8_t j = 0; j < 4; ++j) { s.minUs[j] = 1475; s.maxUs[j] = 1525; }
  FakeJogIO io; ManualJogController c(s, io); prepare(c);
  const size_t writes = io.writes.size();
  assert(!c.jog(0, 100, 0, true, true) && c.error() == JogError::Range);
  assert(c.state() == JogState::Holding && c.commandedPose().us[0] == 1500);
  assert(io.writes.size() == writes);
  assert(c.jog(0, 25, 0, true, true));
  for (uint32_t t = 20; t <= 260; t += 20) c.tick(t, true, true);
  assert(c.state() == JogState::Holding && c.commandedPose().us[0] == 1525);
  assert(!c.jog(0, 1, 260, true, true));
  assert(!c.jog(0, -100, 260, true, true));
  assert(c.jog(0, -25, 260, true, true));
  for (uint32_t t = 280; t <= 520; t += 20) c.tick(t, true, true);
  assert(c.state() == JogState::Holding && c.commandedPose().us[0] == 1500);
  for (const JointPose& p : io.writes) for (uint8_t j = 0; j < 4; ++j) {
    assert(p.us[j] >= 1475 && p.us[j] <= 1525);
    if (j != 0) assert(p.us[j] == 1500);
  }
}
void patientLeaseExpiresAndCannotResume() {
  // Both ordinary time and a 5-minute wait across uint32_t rollover.
  for (uint32_t start : {0U, UINT32_MAX - 150000U}) {
    JogSettings s = patientSettings(); FakeJogIO io; ManualJogController c(s, io); prepare(c, start);
    assert(c.holdRemainingMs(start) == 300000 && c.sessionRemainingMs(start) == 1800000);
    for (uint32_t elapsed = 200; elapsed < 300000; elapsed += 200)
      c.tick(start + elapsed, true, true);
    c.tick(start + 299999, true, true);
    assert(c.state() == JogState::Holding && c.outputsOn());
    assert(c.holdRemainingMs(start + 299999) == 1);
    assert(c.sessionRemainingMs(start + 299999) == 1500001);
    c.tick(start + 300000, true, true);
    assert(c.state() == JogState::Disarmed && c.error() == JogError::LeaseExpired);
    assert(io.detaches == 1 && !c.outputsOn() && !c.seedPresent());
    assert(c.holdRemainingMs(start + 300000) == 0 && c.sessionRemainingMs(start + 300000) == 0);
    assert(!c.hold(start + 300000, true, true));
    c.tick(start + 300020, true, true);
    assert(!c.outputsOn() && io.enables == 1);
    assert(!c.arm(start + 300020, true, true) && c.error() == JogError::Seed);
  }
}
void renewalCannotBypassThirtyMinuteSession() {
  JogSettings s = patientSettings(); FakeJogIO io; ManualJogController c(s, io);
  const uint32_t start = UINT32_MAX - 1000U;
  prepare(c, start);
  for (uint32_t elapsed = 200; elapsed < 1800000; elapsed += 200) {
    c.tick(start + elapsed, true, true);
    // Renew well before each 5-minute idle deadline, including after rollover.
    if (elapsed % 240000 == 0) {
      assert(c.hold(start + elapsed, true, true));
      assert(c.holdRemainingMs(start + elapsed) == 300000);
      assert(c.sessionRemainingMs(start + elapsed) == 1800000 - elapsed);
    }
    assert(c.state() == JogState::Holding && c.outputsOn());
  }
  c.tick(start + 1799999, true, true);
  assert(c.state() == JogState::Holding && c.outputsOn());
  assert(c.sessionRemainingMs(start + 1799999) == 1);
  assert(c.holdRemainingMs(start + 1799999) == 180001);
  c.tick(start + 1800000, true, true);
  assert(c.state() == JogState::Fault && c.error() == JogError::SessionExpired);
  assert(io.detaches == 1 && !c.outputsOn() && !c.seedPresent());
  assert(c.holdRemainingMs(start + 1800000) == 0 && c.sessionRemainingMs(start + 1800000) == 0);
  assert(!c.hold(start + 1800000, true, true));
  c.tick(start + 1800020, true, true);
  assert(!c.outputsOn() && io.enables == 1);
  c.reset();
  assert(!c.arm(start + 1800020, true, true) && c.error() == JogError::Seed);
}
void absolutePoseIsAtomicAndNotQueued() {
  JogSettings s = travelSettings(); FakeJogIO io; ManualJogController c(s, io);
  const JointPose target = {{1800,1200,1550,2100}};
  assert(!c.movePose(target,0,true,true) && io.writes.empty() && io.enables == 0);
  prepare(c); c.tick(100,true,true);
  for (uint8_t joint = 0; joint < 4; ++joint) {
    for (uint16_t outside : {uint16_t(499),uint16_t(2501)}) {
      JointPose invalid = target; invalid.us[joint] = outside;
      assert(!c.movePose(invalid,100,true,true) && c.error() == JogError::Range);
      assert(c.state() == JogState::Holding && io.writes.size() == 1);
      for (uint8_t j = 0; j < 4; ++j) assert(c.commandedPose().us[j] == 1500);
      assert(c.holdRemainingMs(100) == 299900); // Rejected targets do not renew.
    }
  }
  assert(!c.jog(0,101,100,true,true)); // MOVE does not enlarge JOG steps.
  assert(c.movePose(target,100,true,true));
  assert(c.state() == JogState::Jogging && io.writes.size() == 1);
  assert(!c.movePose(seed,100,true,true) && c.error() == JogError::Busy);
  assert(!c.jog(0,1,100,true,true) && c.error() == JogError::Busy);
  assert(!c.hold(100,true,true));
  c.tick(119,true,true); assert(io.writes.size() == 1);
  for (uint32_t t = 120; t <= 6100; t += 20) c.tick(t,true,true);
  assert(c.state() == JogState::Holding && c.error() == JogError::None);
  for (uint8_t j = 0; j < 4; ++j) assert(c.commandedPose().us[j] == target.us[j]);
  c.tick(6200,true,true);
  const size_t writes = io.writes.size();
  assert(c.movePose(target,6200,true,true));
  assert(c.state() == JogState::Holding && io.writes.size() == writes);
  assert(c.holdRemainingMs(6200) == 300000);
  c.tick(6300,true,true); assert(io.writes.size() == writes); // No queued return.
  s.rateUsPerSecond = 0;
  assert(!c.movePose(seed,6300,true,true) && c.error() == JogError::Config);
  assert(c.state() == JogState::Fault && !c.outputsOn());
}
void absolutePoseIsSynchronousAndRateLimited() {
  const JointPose target = {{2101,1200,1600,1500}};
  // Durations not divisible by 20 ms, fractional channel changes and rollover.
  for (uint16_t rate : {uint16_t(50),uint16_t(75),uint16_t(100),uint16_t(500),uint16_t(1000)}) {
    for (uint32_t start : {0U,UINT32_MAX-500U}) {
      JogSettings s = travelSettings(); s.rateUsPerSecond = rate;
      FakeJogIO io; ManualJogController c(s,io); prepare(c,start);
      assert(c.movePose(target,start,true,true));
      const uint32_t duration = (601000U+rate-1U)/rate;
      for (uint32_t elapsed = 20; elapsed < duration+20; elapsed += 20) {
        c.tick(start+elapsed,true,true);
        const uint32_t progress = elapsed < duration ? elapsed : duration;
        for (uint8_t j = 0; j < 4; ++j) {
          const int32_t delta = int32_t(target.us[j])-1500;
          const uint32_t magnitude = uint32_t(delta < 0 ? -delta : delta);
          const int32_t advance = int32_t((uint64_t(magnitude)*progress+duration/2U)/duration);
          assert(c.commandedPose().us[j] == uint16_t(1500+(delta < 0 ? -advance : advance)));
          if (io.writes.size() > 1) {
            const int32_t step = int32_t(io.writes.back().us[j])-io.writes[io.writes.size()-2].us[j];
            assert(step >= -int32_t((rate*20U+999U)/1000U) &&
                   step <= int32_t((rate*20U+999U)/1000U));
          }
        }
        assert(c.state() == (progress == duration ? JogState::Holding : JogState::Jogging));
      }
      assert(c.state() == JogState::Holding);
      for (uint8_t j = 0; j < 4; ++j) assert(c.commandedPose().us[j] == target.us[j]);
    }
  }
  // Delayed accepted ticks advance at most 20 nominal ms, never catch up.
  JogSettings s = travelSettings(); FakeJogIO io; ManualJogController c(s,io); prepare(c);
  const JointPose irregularTarget = {{1600,1450,1525,1500}};
  assert(c.movePose(irregularTarget,0,true,true));
  uint32_t now = 0;
  for (uint32_t update = 1; update <= 50; ++update) {
    now += update % 2 ? 50 : 200; c.tick(now,true,true);
    assert(c.commandedPose().us[0] == 1500+2*update);
    assert(c.commandedPose().us[1] == 1500-update);
    assert(c.commandedPose().us[2] == 1500+(update+1)/2);
    assert(c.commandedPose().us[3] == 1500);
    assert(c.state() == (update == 50 ? JogState::Holding : JogState::Jogging));
  }
  assert(now == 6250); // Nominally 1000 ms, safely slower after delayed ticks.
}
void absolutePoseStopsAndDoesNotResume() {
  const JointPose target = {{2100,1100,1900,2300}};
  for (uint8_t interruption = 0; interruption < 6; ++interruption) {
    JogSettings s = travelSettings(); FakeJogIO io; ManualJogController c(s,io); prepare(c);
    assert(c.movePose(target,0,true,true));
    for (uint32_t t = 20; t <= 100; t += 20) c.tick(t,true,true);
    const size_t writes = io.writes.size();
    if (interruption == 0) c.tick(120,false,true);
    else if (interruption == 1) c.tick(120,true,false);
    else if (interruption == 2) c.estop();
    else if (interruption == 3) c.disarm();
    else if (interruption == 4) c.tick(351,true,true);
    else { io.writeOkay = false; c.tick(120,true,true); }
    assert(!c.outputsOn() && !c.seedPresent() && io.detaches == 1);
    assert(io.writes.size() == writes + (interruption == 5 ? 1U : 0U));
    const size_t stoppedWrites = io.writes.size();
    c.tick(370,true,true); c.tick(390,true,true);
    assert(io.writes.size() == stoppedWrites && io.enables == 1);
    assert(!c.movePose(target,390,true,true));
    c.reset(); assert(!c.arm(390,true,true) && c.error() == JogError::Seed);
  }
  // Permission checked at command acceptance, not only on later ticks.
  JogSettings s = travelSettings(); FakeJogIO io; ManualJogController c(s,io); prepare(c);
  assert(!c.movePose(target,0,false,true));
  assert(c.error() == JogError::Permit && !c.outputsOn() && io.writes.size() == 1);
}
void absolutePoseCannotExtendLeaseOrSession() {
  const JointPose target = {{2500,500,2300,2100}};
  for (uint8_t deadline = 0; deadline < 2; ++deadline) {
    JogSettings s = travelSettings(); s.holdLeaseMs = 1000; s.maxSessionMs = 2000;
    FakeJogIO io; ManualJogController c(s,io); prepare(c);
    if (deadline) {
      for (uint32_t t = 20; t <= 1500; t += 20) {
        c.tick(t,true,true);
        if (t % 500 == 0) assert(c.hold(t,true,true));
      }
    }
    const uint32_t began = deadline ? 1500 : 0;
    assert(c.movePose(target,began,true,true));
    const uint32_t expires = deadline ? 2000 : 1000;
    for (uint32_t t = began+20; t < expires; t += 20) {
      c.tick(t,true,true); assert(c.state() == JogState::Jogging);
    }
    c.tick(expires,true,true);
    assert(c.error() == (deadline ? JogError::SessionExpired : JogError::LeaseExpired));
    assert(c.state() == (deadline ? JogState::Fault : JogState::Disarmed));
    assert(!c.outputsOn() && !c.seedPresent() && io.detaches == 1);
    const size_t writes = io.writes.size();
    c.tick(expires+20,true,true); assert(io.writes.size() == writes && io.enables == 1);
  }
}
int main() {
  defaultsAndExplicitSeed(); boundedSingleJointAndNoQueue(); leaseAndSessionAreFinite();
  physicalLossAndOutputs(); timingAndEstop();
  extendedSettingsRemainBounded(); largerStepsStaySmoothAndSingleJoint();
  largerPermissionDoesNotExpandJointLimits(); patientLeaseExpiresAndCannotResume();
  renewalCannotBypassThirtyMinuteSession();
  absolutePoseIsAtomicAndNotQueued(); absolutePoseIsSynchronousAndRateLimited();
  absolutePoseStopsAndDoesNotResume(); absolutePoseCannotExtendLeaseOrSession();
  std::puts("Manual jog: 14 suites passed (JOG and absolute MOVE; fake outputs only)");
}
