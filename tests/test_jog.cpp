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
int main() {
  defaultsAndExplicitSeed(); boundedSingleJointAndNoQueue(); leaseAndSessionAreFinite();
  physicalLossAndOutputs(); timingAndEstop();
  std::puts("Manual jog: 5 suites passed (fake outputs only; no physical hardware)");
}
